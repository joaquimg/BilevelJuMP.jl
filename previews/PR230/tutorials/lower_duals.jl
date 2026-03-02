# # Dual variables of the lower level

# This is a quick introduction to modeling and solving bilevel optimization
# with BilevelJuMP.

# This modeling feature enables the implementation of workflows where one
# (or more) of the upper-level variables is the dual of a lower-level
# constraint. In particular, in the energy sector, it is common to model the
# energy prices as the dual variable associated with the energy demand
# equilibrium constraint. One example of an application that uses this feature
# is [Fanzeres et al. (2019)](https://doi.org/10.1016/j.ejor.2018.07.027),
# which focuses on strategic bidding in
# auction-based energy markets. A small and simplified example of the modeled
# problem would be the model:

# ```math
# \begin{align}
#     &\max_{\lambda, q_S} \quad \lambda \cdot g_S \\
#     &\textit{s.t.} \quad 0 \leq q_S \leq 100\\
#     &\hspace{28pt} (g_S, \lambda) \in \arg\min_{g_S, g_{1}, g_{2}, g_D} 50 g_{R1} + 100  g_{R2} + 1000 g_{D}\\
#             & \hspace{70pt} \textit{s.t.} \quad g_S \leq q_S \\
#             & \hspace{88pt} \quad  0 \leq g_S \leq 100 \\
#             & \hspace{88pt}\quad  0 \leq g_{1} \leq 40 \\
#             & \hspace{88pt}\quad  0 \leq g_{2} \leq 40 \\
#             & \hspace{88pt}\quad  0 \leq g_{D} \leq 100 \\
#     & \hspace{88pt}\quad  g_S + g_{1} + g_{2} + g_D = 100 \quad  : \quad \lambda \label{eq-dual-lambda}
# \end{align}
# ```

# Where $\lambda$ is the dual of the load balance constraint
# (last constraint in the lower part),
# $g_S$, $g_{1}$, $g_2$ represent the generation of
# the strategic bidder and from two other (non-strategic) plants.
# $g_D$ represents the deficit in generation.
# Finally, $q_S$ is the quantity bid optimized by the strategic generator.

# load packages

using BilevelJuMP
using Ipopt
using QuadraticToBinary
using HiGHS

# BilevelJuMP.jl allows users to implement similar models using the
# function `DualOf` that binds a new variable in the upper level
# to an existing constraint in the lower level.
# The model can be written as:

model = BilevelModel()
@variable(Upper(model), 0 <= qS <= 100)
@variable(Lower(model), 0 <= gS <= 100)
@variable(Lower(model), 0 <= gR1 <= 40)
@variable(Lower(model), 0 <= gR2 <= 40)
@variable(Lower(model), 0 <= gD <= 100)
@objective(Lower(model), Min, 50gR1 + 100gR2 + 1000gD)
@constraint(Lower(model), gS <= qS)
@constraint(Lower(model), demand_equilibrium, gS + gR1 + gR2 + gD == 100)
@variable(Upper(model), lambda, DualOf(demand_equilibrium))
@objective(Upper(model), Max, lambda*gS)

# ## NLP solution

# This model, can be solved by selecting a reformulation and a solver.
# Here we select Strong-Duality reformulation, the Ipopt solver and call
# optimizes to perform the reformulation and solve it.

BilevelJuMP.set_mode(model, BilevelJuMP.StrongDualityMode())
set_optimizer(model, Ipopt.Optimizer)
optimize!(model)

# ## MIP solution

# It is also possible to solve such problem by using a MIP formulation.
# The main issue is the product of variable in the upper level objective.
# However, this can be easily handled by using the package
# `QuadraticToBinary.jl` for automatic binary expansions.
# Because binary expansions require bounds on variables,
# we add the following lines:

set_lower_bound(lambda, 0.0)
set_upper_bound(lambda, 1000.0)

# Then, as before, we set a solver
# (now HiGHS with the `QuadraticToBinary.jl` wrapper) and a solution method
# (now Fortuny-Amat and McCarl):

set_optimizer(model,
    ()->QuadraticToBinary.Optimizer{Float64}(HiGHS.Optimizer()))
BilevelJuMP.set_mode(model,
    BilevelJuMP.FortunyAmatMcCarlMode(dual_big_M = 100))
optimize!(model)

# ## More on `DualOf` usage

# You might have a problem where you want duals of a vector of constraints like:

@constraint(Lower(model), reserves[i=1:3], (40 - gR1) + (40 - gR2) == 10 * i)

# then you can do

@variable(Upper(model), reserve_dual[i=1:3], DualOf(reserves[i]))

# or use anonymous variables

my_duals = []
for i in 1:3
    var = @variable(Upper(model), variable_type = DualOf(reserves[i]))
    push!(my_duals, var)
end
my_duals # a vector of anonimous variables
