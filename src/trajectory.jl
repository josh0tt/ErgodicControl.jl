######################################################################
# trajectory.jl
#
# handles the generation of ergodic trajectories
######################################################################

using StatsBase: WeightVec, sample


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

	# The descender determines how much to descend at each step
	descender::Descender

	# dynamics stuff
	dynamics::Dynamics

	function TrajectoryManager(x0::Vector{Float64}, h::Float64, N::Int)
		return TrajectoryManager(x0, h, N, RandomInitializer())
	end
	function TrajectoryManager(x0::Vector{Float64}, h::Float64, N::Int, i::Initializer)
		tm = new()

		# needed for all trajectories
		tm.N = N
		tm.h = h
		tm.T = N*h
		tm.x0 = deepcopy(x0)

		# needed for ergodic trajectories
		tm.Q = eye(2)
		# trying a higher cost now 2/07/2017
		#tm.R = 0.01 * eye(2)
		tm.R = 0.01 * eye(2)
		tm.q = 1.0
		tm.max_iters = 30
		tm.initializer = i
		tm.descender = InverseRootStep(1.0)
		# I've found InverseRootStep(0.15) to work well

		# dynamics stuff
		tm.dynamics = LinearDynamics(eye(2), tm.h*eye(2))

		return tm
	end
end


"""
`dynamics!(tm::TrajectoryManager, A::Matrix{Float64}, B::Matrix{Float64})`

`dynamics!(tm::TrajectoryManager, d_type::String)`

Sets the dynamics for `tm`.
This includes fields `tm.A`, `tm.B`, `tm.n` and `tm.m`.
Dynamics are assumed to be linear and constant.

User can optionally pass in string `d_type`.
If this string is "double integrator", double integrator dynamics will be made.

If the new `tm.n` (number of state dimensions) is greater than the length of `tm.x0`, `tm.x0` will be padded with zeros until it is of length `tm.n`.

"""
function dynamics!(tm::TrajectoryManager, A::Matrix{Float64}, B::Matrix{Float64})
	ld = LinearDynamics(A, B)
	tm.dynamics = ld
	while length(tm.x0) < ld.n
		push!(tm.x0, 0.)
	end
end

function dynamics!(tm::TrajectoryManager, d_type::String)
	if d_type == "double integrator"
		A = [1 0 tm.h 0; 0 1 0 tm.h; 0 0 1 0; 0 0 0 1]
		B = [0 0; 0 0; tm.h 0; 0 tm.h]
		dynamics!(tm, A, B)
	elseif d_type == "dubins"
		tm.dynamics = DubinsDynamics(1.0, 1.0)
	else
		error("Invalid d_type provided!")
	end
end


function initialize(em::ErgodicManager, tm::TrajectoryManager)
	initialize(tm.initializer, em, tm)
end

# TODO: change rand() call so we can account for various
function initialize(ri::RandomInitializer, em::ErgodicManager, tm::TrajectoryManager)
	xd = [[tm.x0[1], tm.x0[2]] for i = 1:tm.N+1]
	points = Array(Vector{Float64}, tm.N)
	for i = 1:tm.N
		points[i] = [rand(), rand()]
	end

	tsp_nn!(xd, points)

	ud = compute_controls(xd, tm.h)

	return xd, ud
end
export compute_controls



# Assumes we our domain is 2d
# corner 1 = 0,0
# corner 2 = 1,0
# corner 3 = 0,1
# corner 4 = 1,1
function initialize(ci::CornerInitializer, em::ErgodicManager, tm::TrajectoryManager)

	# dimensionality of state space
	x0 = tm.x0
	n = length(x0)

	# first corner: 0,0
	dx1 = 0.0 - x0[1]
	dy1 = 0.0 - x0[2]
	dc1 = sqrt(dx1*dx1 + dy1*dy1)

	# second corner: 1,0
	dx2 = 1.0 - x0[1]
	dy2 = 0.0 - x0[2]
	dc2 = sqrt(dx2*dx2 + dy2*dy2)

	# third corner: 0,1
	dx3 = 0.0 - x0[1]
	dy3 = 1.0 - x0[2]
	dc3 = sqrt(dx3*dx3 + dy3*dy3)

	# fourth corner: 1,1
	dx4 = 1.0 - x0[1]
	dy4 = 1.0 - x0[2]
	dc4 = sqrt(dx4*dx4 + dy4*dy4)

	bi = indmax([dc1,dc2,dc3,dc4])

	bc = [0.,0.]
	if bi == 2
		bc = [1.,0.]
	elseif bi == 3
		bc = [0.,1.]
	elseif bi == 4
		bc = [1.,1.]
	end

	# could now do point initializer to the selected corner
	return initialize(PointInitializer(bc), em, tm)
end


function initialize(ci::ConstantInitializer, em::ErgodicManager, tm::TrajectoryManager)
	xd = Array(Vector{Float64}, tm.N+1)
	ud = Array(Vector{Float64}, tm.N)
	xd[1] = deepcopy(tm.x0)
	ud[1] = deepcopy(ci.action)
	for i = 1:(tm.N-1)
		# 2/07/2017: need to compute true A
		#A,B = linearize(tm, xd[i], ud[i])
		#display(A)
		#xd[i+1] = A*xd[i] + B*ud[i]
		xd[i+1] = forward_euler(tm, xd[i], ud[i])
		ud[i+1] = deepcopy(ci.action)
	end
	A,B = linearize(tm, xd[tm.N], ud[tm.N])
	xd[tm.N+1] = A*xd[tm.N] + B*ud[tm.N]

	return xd, ud
end

# TODO: actually finish this
function initialize(cci::CornerConstantInitializer, em::ErgodicManager, tm::TrajectoryManager)
	xd = Array(Vector{Float64}, tm.N+1)
	ud = Array(Vector{Float64}, tm.N)
	xd[1] = deepcopy(tm.x0)
	ud[1] = deepcopy(ci.action)
	for i = 1:(tm.N-1)
		xd[i+1] = tm.A*xd[i] + tm.B*ud[i]
		ud[i+1] = deepcopy(ci.action)
	end
	xd[tm.N+1] = tm.A*xd[tm.N] + tm.B*ud[tm.N]

	return xd, ud
end


function initialize(si::SampleInitializer, em::ErgodicManager, tm::TrajectoryManager)
	xd = Array(Vector{Float64}, tm.N+1)
	points = Array(Vector{Float64}, tm.N)
	xd[1] = deepcopy(tm.x0)
	ud = Array(Vector{Float64}, tm.N)
	bin_size = (em.bins, em.bins)

	# first sample N points from e.phi
	weights = WeightVec(vec(em.phi))
	for n = 1:tm.N
		xi, yi = ind2sub(bin_size, sample(weights))
		points[n] = [em.cell_size*(xi-.5), em.cell_size*(yi-.5)]
	end

	# find a short path heuristically
	#tsp_rand!(xd, points)
	tsp_nn!(xd, points)

	# compute controls
	ud = compute_controls(xd, tm.h)

	return xd, ud
end


# computes controls from a trajectory
# TODO: really, this is a general tool useful for other code
#  it should really go somewhere else
function compute_controls(xd::VV_F, h::Float64)
	N = length(xd) - 1
	ud = Array(Vector{Float64}, N)
	for n = 1:N
		ud[n] = (xd[n+1] - xd[n]) / h
	end
	
	return ud
end

"""
controls2trajectory(tm::TrajectoryManager, ud::VV_F)

computes trajectory from controls (and initial position)
"""
function controls2trajectory(tm::TrajectoryManager, ud::VV_F)
	N = length(ud)
	xd = Array(Vector{Float64}, N+1)

	xd[1] = deepcopy(tm.x0)
	for i = 1:tm.N
		xd[i+1] = tm.dynamics.A*xd[i] + tm.dynamics.B*ud[i]
	end

	return xd
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


# simply takes the points as they are
# this creates a shitty trajectory
function tsp_rand!(xd::VV_F, points::VV_F)
	n = 1
	for p in points
		xd[n+1] = p
		n += 1
	end
end

# creates a sample_trajectory
function sample_trajectory(em::ErgodicManager, tm::TrajectoryManager)
	return initialize(SampleInitializer(), em, tm)
end


export greedy_trajectory
function greedy_trajectory(em::ErgodicManager, tm::TrajectoryManager)
	return initialize(GreedyInitializer(), em, tm)
end
function initialize(gi::GreedyInitializer, em::ErgodicManager, tm::TrajectoryManager)
	d_rate = sum(em.phi)/tm.N
	num_cells = em.bins*em.bins
	total_info = 0.0
	xd = Array(Vector{Float64}, tm.N+1)
	xd[1] = deepcopy(tm.x0)
	temp_phi = deepcopy(em.phi)
	size_tuple = (em.bins, em.bins)
	for n = 1:tm.N
		bi = indmax(temp_phi)
		xi, yi = ind2sub(size_tuple, bi)
		xd[n+1] = [(xi-0.5)*em.cell_size, (yi-0.5)*em.cell_size]
		temp_phi[bi] -= min(temp_phi[bi], d_rate)
	end
	ud = compute_controls(xd, tm.h)
	return xd,ud
end


# moves to a point with a constant control input
function initialize(initializer::PointInitializer, em::ErgodicManager, tm::TrajectoryManager)

	# compute the direction and action we most go towards
	dx = initializer.xd[1] - tm.x0[1]
	dy = initializer.xd[2] - tm.x0[2]
	#x_step = (initializer.xd[1] - tm.x0[1]) / (tm.N * tm.h)
	#y_step = (initializer.xd[2] - tm.x0[2]) / (tm.N * tm.h)
	#u = [x_step, y_step]

	# if we are double integrator, only apply the input once
	ud = Array(Vector{Float64}, tm.N)
	if tm.dynamics.n > 2
		den = tm.h * tm.h * (tm.N-1)
		ud[1] = [dx/den, dy/den]
		for i = 1:(tm.N-1)
			ud[i+1] = zeros(2)
		end
	else  # single integrator
		# TODO: can't I just make this tm.T?
		den = tm.h * tm.N
		u = [dx / den, dy / den]
		for i = 1:tm.N
			ud[i] = deepcopy(u)
		end
	end

	xd = controls2trajectory(tm, ud)

	return xd, ud
end

# moves to a point with a constant control input
function initialize(initializer::DirectionInitializer, em::ErgodicManager, tm::TrajectoryManager)
	xd = Array(Vector{Float64}, tm.N+1)
	ud = Array(Vector{Float64}, tm.N)

	# compute the direction and action we most go towards
	dx = initializer.xd[1] - tm.x0[1]
	dy = initializer.xd[2] - tm.x0[2]
	u = initializer.mag * [dx, dy] / sqrt(dx*dx + dy*dy)

	xd[1] = deepcopy(tm.x0)
	ud[1] = deepcopy(u)
	for i = 1:(tm.N-1)
		xd[i+1] = tm.A*xd[i] + tm.B*ud[i]
		ud[i+1] = deepcopy(u)
	end
	xd[tm.N+1] = tm.A*xd[tm.N] + tm.B*ud[tm.N]

	return xd, ud
end

function linearize(tm::TrajectoryManager, x::Vector{Float64}, u::Vector{Float64})
	return linearize(tm.dynamics, x, u, tm.h)
end
function forward_euler(tm::TrajectoryManager, x::Vector{Float64}, u::Vector{Float64})
	forward_euler(tm.dynamics, x, u, tm.h)
end
