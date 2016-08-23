module ErgodicControl

# export functions I've made
export ErgodicManager, phik!, reconstruct, decompose
export ergodic_score, control_score, total_score
export centroid, covariance
export TrajectoryManager, dynamics!, make_trajectory, sample_trajectory
export kmeans_trajectory, random_trajectory
export assign_step, mean_step
export Initializer, RandomInitializer, CornerInitializer, ConstantInitializer, initialize
export clerc_trajectory

# to make some things easier
typealias VV_F   Vector{Vector{Float64}}   # vector of vector of floats
typealias V_T2F  Vector{NTuple{2,Float64}} # vector of tuples of 2 floats
typealias VMF64  Vector{Matrix{Float64}}   # vector of matrix of floats
typealias T2F      NTuple{2, Float64}    # x, y

include("math.jl")

include("ergodicity.jl")
include("initializer.jl")
include("trajectory.jl")
include("scoring.jl")
include("clerc_trajectory.jl")
include("max_trajectory.jl")
include("sample_trajectory.jl")
include("kmeans_trajectory.jl")
include("plots.jl")
include("gif.jl")

end # module
