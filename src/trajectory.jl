######################################################################
# trajectory.jl
#
# handles the generation of ergodic trajectories
######################################################################

type TrajectoryManager

	# needed for all trajectories
	N::Int
	h::Float64
	x0::Vector{Float64}
	T::Float64
	
	# needed for ergodic trajectories
	Q::Matrix{Float64}
	R::Matrix{Float64}
	q::Float64
	max_iters::Int
	initializer::Initializer

	# dynamics stuff
	n::Int
	m::Int
	A::Matrix{Float64}
	B::Matrix{Float64}

	function TrajectoryManager(x0::Vector{Float64}, h::Float64, N::Int)
		tm = new()

		# needed for all trajectories
		tm.N = N
		tm.h = h
		tm.T = N*h
		tm.x0 = deepcopy(x0)

		# needed for ergodic trajectories
		tm.Q = eye(2)
		tm.R = 0.01 * eye(2)
		tm.q = 1.0
		tm.max_iters = 30
		tm.initializer = RandomInitializer()

		# dynamics stuff
		tm.n = 2
		tm.m = 2
		tm.A = eye(2)
		tm.B = tm.h * eye(2)

		return tm
	end
end

"""
`dynamics!(tm::TrajectoryManager, A::Matrix{Float64}, B::Matrix{Float64})`

Sets the dynamics for `tm`.
This includes fields `tm.A`, `tm.B`, `tm.n` and `tm.m`.
Dynamics are assumed to be linear and constant.
"""
function dynamics!(tm::TrajectoryManager, A::Matrix{Float64}, B::Matrix{Float64})
	tm.n, tm.m = size(B)
	tm.A = deepcopy(A)
	tm.B = deepcopy(B)
end


"""
`decompose(em, traj::VV_F)`

`decompose(em, traj::V_T2F)`

Decomposes a set of positions into a set of `ck` Fourier coefficients.
"""
function decompose(em::ErgodicManager, traj::VV_F)
	traj2 = [(traj[i][1], traj[i][2]) for i = 1:length(traj)]
	return decompose(em, traj2)
end
function decompose(em::ErgodicManager, traj::V_T2F)
	K = em.K
	N = length(traj)-1
	ck = zeros(K+1, K+1)
	for k1 = 0:K
		kpiL1 = k1 * pi / em.L
		for k2 = 0:K
			kpiL2 = k2 * pi / em.L
			hk = em.hk[k1+1, k2+1]
			fk_sum = 0.0
			# now loop over time
			for n = 0:N-1
				xn = traj[n+1]
				fk_sum += cos(kpiL1 * xn[1])  * cos(kpiL2 * xn[2])
			end
			ck[k1+1, k2+1] = fk_sum / (hk * N)
		end
	end
	return ck
end


"""
`ergodic_score(em, traj::V_T2F)`

First breaks down the trajectory into components ck.
"""
function ergodic_score(em::ErgodicManager, traj::V_T2F)
	ck = decompose(em, traj)
	return ergodic_score(em, ck)
end
function ergodic_score(em::ErgodicManager, traj::VV_F)
	ck = decompose(em, traj)
	return ergodic_score(em, ck)
end
function ergodic_score(em::ErgodicManager, ck::Matrix{Float64})
	val = 0.0
	for k1 = 0:em.K
		for k2 = 0:em.K
			d = em.phik[k1+1,k2+1] - ck[k1+1,k2+1]
			val += em.Lambdak[k1+1,k2+1] * d * d
		end
	end
	return val
end

"""
`control_score(ud::VV_F, R, h)`

Assumes only non-zero elements of `R` are corners.
"""
function control_score(ud::VV_F, R::Matrix{Float64}, h::Float64)
	N = length(ud) - 1
	cs = 0.0
	for ui in ud[1:end-1]
		cs += R[1,1] * ui[1] * ui[1]
		cs += R[2,2] * ui[2] * ui[2]
	end
	return 0.5 * h * cs
end
control_score(ud::VV_F) = control_score(ud, eye(2), 1.0)


"""
`total_score(em, xd::VV_F, ud::VV_F, T::Float64)`

Computes the total score `q*ergodic_score + sum_n h/2 un'Rn un`

Currently assumes `q = 1.0` and `R = 0.01 * eye(2)`
"""
# TODO: actually get q and R from the correct place 
function total_score(em::ErgodicManager, xd::VV_F, ud::VV_F, T::Float64)
	q = 1.0
	R = 0.01 * eye(2)
	N = length(xd) - 1
	h = T/N
	return q * ergodic_score(em, xd) + control_score(ud, R, h)
end
# TODO: let's not make this so shitty...
function total_score(em::ErgodicManager, xd::VV_F, ud::VV_F, zd::VV_F, vd::VV_F, alpha::Float64, T::Float64)
	xd2 = deepcopy(xd)
	ud2 = deepcopy(ud)
	for i = 1:length(xd2)
		xd2[i][1] += alpha * zd[i][1]
		xd2[i][2] += alpha * zd[i][2]

		ud2[i][1] += alpha * vd[i][1]
		ud2[i][2] += alpha * vd[i][2]
	end
	return total_score(em, xd2, ud2, T)
end


# currently, I just move in a random direction
# perhaps find direction to mean and move towards it
function initialize_trajectory(N::Int, h::Float64, x0::T2F)
	xd = [[x0[1], x0[2]] for i = 1:N+1]
	#ud = [.01*ones(2) for i = 1:N+1]
	#ud = [[.01,0.] for i = 1:N+1]
	ud = [[.001,0.001] for i = 1:N+1]
	for i = 2:N+1
		xd[i][1] = xd[i-1][1] + h*ud[i-1][1]
		xd[i][2] = xd[i-1][2] + h*ud[i-1][2]
	end
	return xd, ud
end


# computes controls from a trajectory
# TODO: really, this is a general tool useful for other code
#  it should really go somewhere else
function compute_controls(xd::VV_F, h::Float64)
	N = length(xd) - 1
	ud = Array(Vector{Float64}, N+1)
	for n = 1:N
		ud[n] = (xd[n+1] - xd[n]) / h
	end
	ud[N+1] = ud[N]   # this one doesn't matter
	
	return ud
end

# uses nearest neighbors to heuristically solve tsp
# tsp is traveling salesman problem
function tsp_nn!(xd::VV_F, points::VV_F)
	xc = xd[1]
	next_p = xd[1]
	n = 1
	while length(points) > 0
		best_d = Inf
		best_ind = 1
		for (p_ind,p) in enumerate(points)
			r = xc - p 
			d = dot(r, r)
			if d < best_d
				best_d = d
				best_ind = p_ind
			end
		end
		xd[n+1] = points[best_ind]
		deleteat!(points, best_ind)
		n += 1
	end
end



"""
`collect_info(em::ErgodicManager, traj::VV_F, d_rate::Float64)`

`collect_info(em::ErgodicManager, traj::VV_F)`

modifies em.phi according to some submodular.
we don't use the last point in the trajectory

decreases at rate `D/T`
If you spend `h` time there, it is equivalent to `h*D/(h*N) = D/N`

Returns total info picked up (a scalar value).
"""
function collect_info(em::ErgodicManager, traj::VV_F; steps=0)
	N = length(traj) - 1
	D = sum(em.phi)
	d_rate = D/N
	collect_info(em, traj, d_rate, steps=steps)
end

function collect_info(em::ErgodicManager, traj::VV_F, d_rate::Float64; steps=0)
	N = length(traj) - 1
	total_info = 0.0
	if steps != 0
		N = steps
	end
	for n = 0:(N-1)
		xi,yi = find_cell(em, traj[n+1])

		# if there is enough info, grab that shit yo
		info_value = min(em.phi[xi,yi], d_rate)
		em.phi[xi,yi] -= info_value
		total_info += info_value
	end
	return total_info
end
export collect_info

function find_cell(em::ErgodicManager, x::Vector{Float64})
	x1 = round(Int, x[1] / em.cell_size, RoundDown) + 1
	x2 = round(Int, x[2] / em.cell_size, RoundDown) + 1
	if x1 > em.bins; x1 -= 1; end
	if x2 > em.bins; x2 -= 1; end
	return x1, x2
end

# 
function optimal_info(em::ErgodicManager, N::Int)
	d_rate = sum(em.phi)/N
	num_cells = em.bins*em.bins
	total_info = 0.0
	for n = 1:N
		best_i = 0
		for i = 1:num_cells
			if em.phi[i] > d_rate
				best_i = i
				total_info += d_rate
				em.phi[best_i] -= d_rate
				break
			end
		end
		if best_i == 0  # we didn't find a good enough cell,
			# loop over and find max
			best_i = indmax(em.phi)
			total_info += em.phi[best_i]
			em.phi[best_i] = 0.0
		end
	end
	return total_info
end
export optimal_info
