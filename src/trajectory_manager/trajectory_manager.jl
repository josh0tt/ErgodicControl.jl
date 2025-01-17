######################################################################
# trajectory_manager.jl
#
# handles the trajectory manager
######################################################################

using StatsBase: Weights, sample

abstract type Dynamics end
abstract type Initializer end
abstract type Descender end
abstract type IntegrationScheme end


mutable struct TrajectoryManager

	# needed for all trajectories
	N::Int
	h::Float64
	x0::Vector{Float64}

	# Cost functions
	q::Float64
	R::Matrix{Float64}
	Qn::Matrix{Float64}
	Rn::Matrix{Float64}
	barrier_cost::Float64

	initializer::Initializer
	descender::Descender
	dynamics::Dynamics
	int_scheme::IntegrationScheme


	function TrajectoryManager(x0::Vector{Float64}, h::Real, N::Int, i::Initializer=RandomInitializer())
		tm = new()

		# needed for all trajectories
		tm.N = N
		tm.h = h
		tm.x0 = deepcopy(x0)

		# needed for ergodic trajectories
		tm.Qn = [1 0; 0 1]
		tm.q = 1.0
		tm.R = 0.01 * [1 0; 0 1]
		tm.Rn = [1 0; 0 1]
		tm.barrier_cost = 0.
		tm.initializer = i
		tm.descender = ArmijoLineSearch()

		# dynamics stuff
		tm.dynamics = LinearDynamics([1 0; 0 1], tm.h*[1 0; 0 1])
		tm.int_scheme = ForwardEuler()

		return tm
	end
end

#typealias VTM Vector{TrajectoryManager}
const VTM = Vector{TrajectoryManager}



# computes controls from a trajectory
# TODO: really, this is a general tool useful for other code
#  it should go somewhere else
export compute_controls
function compute_controls(xd::VVF, h::Float64)
	N = length(xd) - 1
	ud = Array(Vector{Float64}, N)
	for n = 1:N
		ud[n] = (xd[n+1] - xd[n]) / h
	end

	return ud
end


# creates a sample_trajectory
function sample_trajectory(em::ErgodicManager, tm::TrajectoryManager)
	return initialize(SampleInitializer(), em, tm)
end
