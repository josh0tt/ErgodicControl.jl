######################################################################
# se48.jl
#
# copy of se4.jl
# but, we try caching more intelligently
######################################################################


export ErgodicManagerSE2

"""
`ErgodicManagerSE2(L::Float64, K::Int, bins::Int)`

`ErgodicManagerSE2(example_name::String; K::Int=5, bins::Int=100)`

Valid `example_name` entries are:

* "single gaussian"
* "double gaussian"
"""
mutable struct ErgodicManagerSE2 <: ErgodicManager
    domain::Domain
    M::Int
    N::Int
    P::Int
    phi::Array{Float64,3}				# spatial distribution
    phik::Array{Complex{Float64},3}		# spatial Fourier coefficients
    Lambda::Array{Float64,3}			# constants 

    bessel_cache::Array{Complex{Float64},6}


    function ErgodicManagerSE2(d::Domain, phi::Array{Float64,3}, K::Int=5)
        em = new()
        em.domain = deepcopy(d)
        em.M = K
        em.N = K
        em.P = K
        em.phi = deepcopy(phi)
        em.phik = zeros(em.M+1, em.N+1, em.P)
        em.Lambda = zeros(em.M+1, em.N+1, em.P)

        tic()
        em.bessel_cache = make_bessel_cache(em)
        rar = toq()
        println("cache time = ", rar)

        Lambda!(em)
        decompose!(em)
        return em
    end

    function ErgodicManagerSE2(d::Domain, K::Int=5)
        em = new()
        em.domain = deepcopy(d)
        em.M = K
        em.N = K
        em.P = K
        em.phik = zeros(em.M+1, em.N+1, em.P)
        em.Lambda = zeros(em.M+1, em.N+1, em.P)

        Lambda!(em)
        return em
    end
end

function make_bessel_cache(em)
    cache = zeros(Complex{Float64}, z_cells(em), y_cells(em), x_cells(em), em.P, em.N+1, em.M+1)
    for m = 0:em.M, n = 0:em.N
        for p = 1:em.P
            for xi = 1:x_cells(em), yi = 1:y_cells(em), zi=1:z_cells(em)
                x = x_min(em) + (xi-0.5)*x_size(em)
                y = y_min(em) + (yi-0.5)*y_size(em)
                z = z_min(em) + (zi-0.5)*z_size(em)
                cache[zi,yi,xi,p,n+1,m+1] = F_mnp(m,n,p,x,y,z)
            end
        end
    end
    return cache
end



# fills the matrix Lambda_{m,n,p}
function Lambda!(em::ErgodicManagerSE2)
    for m = 0:em.M, n = 0:em.N, p = 1:em.P
        den_sqrt = (1.0 + m*m + n*n + p*p)
        em.Lambda[m+1, n+1, p] = 1.0 / (den_sqrt * den_sqrt)
    end
end



######################################################################
# Setting and computing phi
######################################################################
function phi!(em::ErgodicManagerSE2, dm::VF, ds::MF)
    # first, generate d
    #d = zeros(em.bins, em.bins, em.bins)
    d = zeros(x_cells(em), y_cells(em), z_cells(em))
    d_sum = 0.0
    for xi = 1:x_cells(em)
		x = x_min(em) + (xi-0.5)*x_size(em)
		println("xi = ", xi)
		for yi = 1:y_cells(em)
			y = y_min(em) + (yi-0.5)*y_size(em)
			for zi = 1:z_cells(em)
				z = z_min(em) + (zi-0.5)*z_size(em)
				d[xi,yi,zi] = my_pdf([x,y,z], dm, ds)
				d_sum += d[xi,yi,zi]
			end
		end
	end
    normalize!(d, em.domain.cell_size)
    em.phi = d
end


######################################################################
# Computing Fourier coefficients
######################################################################
# update the Fourier coefficients based on some distribution
# Here, I assume it is discrete, but I should change this...
# TODO: maybe I should do some bounds checking?
function decompose!(em::ErgodicManagerSE2, d::Array{Float64,3})
    for m = 0:em.M
        #println("m = ", m)
        for n = 0:em.N, p = 1:em.P
            em.phik[m+1,n+1,p] = phi_mnp(em, m, n, p, d)
        end
    end
    em.phi = d
end


# iterate over the state space
function phi_mnp(em::ErgodicManagerSE2, m::Int, n::Int, p::Int, d::Array{Float64,3})
	val = 0.0im
	for xi = 1:x_cells(em), yi = 1:y_cells(em), zi = 1:z_cells(em)
        #val += d[xi,yi,zi] * em.bessel_cache[m+1,n+1,p,xi,yi,zi]*p / (4*pi*pi)
        #val += d[xi,yi,zi] * em.bessel_cache[m+1,n+1,p,xi,yi,zi]
        val += d[xi,yi,zi] * em.bessel_cache[zi, yi, xi, p, n+1, m+1]
	end
	return val * em.domain.cell_size
end

function F_mnp(m::Int, n::Int, p::Int, x::Float64, y::Float64, z::Float64)
    # compute psi, r
    r = sqrt(x*x + y*y)
    psi = atan2(y,x)	# atan(y/x)
    i = float(im)
    return i^(n-m) * exp(i*(m*psi + (n-m)z)) * besselj(m-n, p*r)
end


function decompose(em::ErgodicManagerSE2, traj::VVF)
    N = length(traj)-1
    ck = zeros(Complex{Float64}, em.M+1, em.N+1, em.P)
    for m = 0:em.M, n = 0:em.N, p = 1:em.P
        fk_sum = 0.0im
        # now loop over time
        for i = 0:N-1
            xi = traj[i + 1]
            fk_sum += F_mnp(m,n,p,xi[1],xi[2],xi[3])
        end
        # TODO: check that this is right
        ck[m+1, n+1, p] = fk_sum / N
    end
    return ck
end


# reconstructs from Fourier coefficients in ck
# This comes from last equation in chapter 10.3 of 
#  Engineering Applications of Noncommutative Harmonic Analysis
# TODO: This is a first cut. Things that I can/should probably change
#  * probably compute M, N before P
#  * p = 0 seems to add nothing, don't bother there
#  * is the call to real() necessary?
function reconstruct(em::ErgodicManagerSE2, ck::Array{Complex{Float64},3})

    vals = zeros(x_cells(em), y_cells(em), z_cells(em))
    #vals = zeros(Complex{Float64}, x_cells(em), y_cells(em), z_cells(em))

    for xi = 1:x_cells(em), yi = 1:y_cells(em), zi=1:z_cells(em)
        for m = 0:em.M, n = 0:em.N, p = 1:em.P
            c = ck[m+1,n+1,p]
            #f = conj(em.bessel_cache[m+1,n+1,p,xi,yi,zi])
            f = conj(em.bessel_cache[zi, yi, xi, p, n+1, m+1])
            vals[xi,yi,zi] += real(c * f * p)
            #vals[xi,yi,zi] += c * f * p * p
            #vals[xi,yi,zi] += c * f
        end
	end
	return vals
end
