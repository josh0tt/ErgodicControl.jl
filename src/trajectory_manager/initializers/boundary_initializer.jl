######################################################################
# boundary_initializer.jl
######################################################################

"""
`bi = BoundaryInitializer()`

Takes a trajectory normal to the boundary.
"""
mutable struct BoundaryInitializer <: Initializer end


# Assumes we our domain is 2d
# corner 1 = 0,0
# corner 2 = 1,0
# corner 3 = 0,1
# corner 4 = 1,1

using LinearAlgebra

function outward_normal(points, query_point)
    # Ensure the points form a closed loop
    if !(query_point in points)
        closest_point = points[argmin([norm(points[i] - query_point) for i = 1:length(points)])]
    else
        closest_point = query_point
    end

    query_idx = findall(x-> x==closest_point, points)[1]
    if query_idx == 1
        next_point = points[2]
        prev_point = points[end]
    elseif query_idx == length(points)
        next_point = points[1]
        prev_point = points[end-1]
    else
        next_point = points[query_idx+1]
        prev_point = points[query_idx-1]
    end

    # Compute the vectors from the query point to the adjacent points
    vector_from_prev = abs.(closest_point - prev_point)
    vector_from_next = abs.(closest_point - next_point)

    # Compute the outward normal by averaging the vectors and rotating by 90 degrees
    outward_normal = (vector_from_prev + vector_from_next) / 2

    # Normalize the outward normal
    outward_normal /= norm(outward_normal)

    xmin = minimum([p[1] for p in points])
    xmax = maximum([p[1] for p in points])
    ymin = minimum([p[2] for p in points])
    ymax = maximum([p[2] for p in points])

    x_sign = 1
    if query_point[1] == xmin
        x_sign = -1
    end

    y_sign = 1
    if query_point[2] == ymin
        y_sign = -1
    end

    return [x_sign*outward_normal[2], y_sign*outward_normal[1]]
end


function initialize(bi::BoundaryInitializer, em::ErgodicManager, tm::TrajectoryManager)

	# dimensionality of state space
	x0 = tm.x0
	n = length(x0)

    points = em.xy_points_boundary.points
    points_vec = [[points[i][1], points[i][2]] for i = 1:length(points)]
    normal = outward_normal(points_vec, [x0[1], x0[2]])

	# could now do point initializer to the selected corner
	return initialize(ConstantInitializer([normal[1],normal[2]]), em, tm)
end
