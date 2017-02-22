######################################################################
# plots.jl
######################################################################
using PyPlot

export plot, plot_trajectory
"""
`plot(em::ErgodicManager, xd::VV_F; alpha=1.0, cmap="Greys", show_score=true, right=true, lw=1.0, ms=6.0)`

`plot(em::ErgodicManager; alpha=1.0, cmap="Greys")`

The "Greys" cmap is dark where there is most density.
The "gray" cmap option is light where there is most density.

An `alpha` value closest to 1.0 is darker; less is more transparent.
"""
function plot(em::ErgodicManager, xd::VV_F; alpha=1.0, cmap="Greys", show_score::Bool=true, right::Bool=true, lw::Float64=1.0, ms::Float64=6.0)
	plot_trajectory(xd, lw=lw, ms=ms)
	hold(true)
	plot(em, alpha=alpha, cmap=cmap)
	if show_score
		start_idx = right ? 1 : 0
		es = ergodic_score(em, xd, start_idx)
		title_string = "es = $(round(es,5))"
		title(title_string)
	end
end

function plot(em::ErgodicManager; alpha=1.0, cmap="Greys")
	a = [0,em.L,0,em.L]
	imshow(em.phi', interpolation="none",cmap=cmap,origin="lower",extent=a,vmin=0, alpha=alpha)
	#labels()	# from an old package
	axis(a)
end

# assumes L = 1.0
function plot(mat::Matrix{Float64}; alpha=1.0, cmap="Greys")
	L = 1.0
	a = [0,L,0,L]
	imshow(mat', interpolation="none",cmap=cmap,origin="lower",extent=a,vmin=0, alpha=alpha)
	#labels()	# from an old package
	axis(a)
end

# what other stuff do we need here?
# only marks, colors, etc
function plot_trajectory(xd::VV_F; lw::Float64=1.0, ms::Float64=6.0)
	N = length(xd)
	xvals = zeros(N)
	yvals = zeros(N)
	for i = 1:N
		xvals[i] = xd[i][1]
		yvals[i] = xd[i][2]
	end
	PyPlot.plot(xvals, yvals, ".-", lw=lw, ms=ms)
end