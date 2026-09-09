# # Dual variables of the lower level

# BilevelJuMP supports use of duals of lower-problem constraints as variables
# in the upper problem via the `DualOf()` function.

# `DualOf()` takes a named constraint from the lower problem as an argument.
# It is used in the `expr` argument of the `@variable` JuMP macro:

# ```julia
# @constraint(Lower(model), some_constraint, x <= y)
#
# @variable(Upper(model), lambda, DualOf(some_constraint))
# ```

# ## Example: strategic bidding in an energy market 

# In the energy sector, it is common to model market prices as the dual of the
# demand equilibrium constraint in the lower-level economic dispatch problem.
# One example of this approach is found in
# [Fanzeres et al. (2019)](https://doi.org/10.1016/j.ejor.2018.07.027),
# which focuses on strategic energy offers in auction-based energy markets. A
# simplified example of the model is:

# ```math
# \begin{align}
#     &\max_{\lambda, q_S} \quad \lambda \cdot g_S \\
#     &\textit{s.t.} \quad 0 \leq q_S \leq 100\\
#     &\hspace{28pt} (g_S, \lambda) \in \arg\min_{g_S, g_{R1}, g_{R2}, g_D} 50 g_{R1} + 100 g_{R2} + 1000 g_{D}\\
#             & \hspace{70pt} \textit{s.t.} \quad g_S \leq q_S \\
#             & \hspace{88pt} \quad  0 \leq g_S \leq 100 \\
#             & \hspace{88pt}\quad  0 \leq g_{R1} \leq 40 \\
#             & \hspace{88pt}\quad  0 \leq g_{R2} \leq 40 \\
#             & \hspace{88pt}\quad  0 \leq g_{D} \leq 100 \\
#     & \hspace{88pt}\quad  g_S + g_{R1} + g_{R2} + g_D = 100 \quad  : \quad \lambda \label{eq-dual-lambda}
# \end{align}
# ```

# where:
#  * $S$ is the strategically-bidding asset controlled by the upper-problem
#    agent, where:
#    * $q_S$ is the quantity of generation offered into the market, to be
#      optimized by the upper-problem agent to maximize revenue, and
#    * $g_S$ is the quantity of generation dispatched by the lower-problem
#      system operator, which is no greater than $q_S$
#  * $g_1$ and $g_2$ are the generation of two other non-strategic,
#    price-taking generators;
#  * $g_D$ is the deficit in generation; and
#  * $\lambda$ is the dual of the load balance constraint

# To implement this model in BilevelJuMP, first load the necessary packages:

using BilevelJuMP
using Ipopt
using QuadraticToBinary
using HiGHS

# Instantiate the model and fully describe the lower problem:

model = BilevelModel()

@variable(Upper(model), 0 <= qS <= 100)

@variable(Lower(model), 0 <= gS <= 100)
@variable(Lower(model), 0 <= g1 <= 40)
@variable(Lower(model), 0 <= g2 <= 40)
@variable(Lower(model), 0 <= gD <= 100)

@objective(Lower(model), Min, 50g1 + 100g2 + 1000gD)

@constraint(Lower(model), gS <= qS)
@constraint(Lower(model), demand_equilibrium, gS + g1 + g2 + gD == 100)

# The BilevelJuMP.jl function `DualOf()` binds a new variable in the upper
# level to an existing constraint in the lower level:

@variable(Upper(model), lambda, DualOf(demand_equilibrium))
@objective(Upper(model), Max, lambda*gS)

# ### NLP solution

# This model can be solved by selecting a reformulation and a solver.
# Here we select Strong-Duality reformulation and the Ipopt solver, and call
# `optimize!()` to perform the reformulation and solve it.

BilevelJuMP.set_mode(model, BilevelJuMP.StrongDualityMode())
set_optimizer(model, Ipopt.Optimizer)
optimize!(model)

# ### MIP solution

# It is also possible to solve such problem by using a MIP formulation.
# The main issue is the product of variables in the upper level objective.
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

# ## Using `DualOf()` with vectors of constraints

# `DualOf()` can also be used to create a vector of variables in the upper
# problem based on a vector of named constraints in the lower problem.

# `DualOf()` requires a named constraint as an input, for example:

@constraint(Lower(model), reserves[i=1:3], (40 - g1) + (40 - g2) == 10 * i)

# You can use `DualOf()` with the built-in JuMP method for creating vectors of
# named variables:

@variable(Upper(model), reserve_dual[i=1:3], DualOf(reserves[i]))

# You can also use `DualOf()` to create a vector of anonymous variables:

my_duals = []
for i in 1:3
    var = @variable(Upper(model), variable_type = DualOf(reserves[i]))
    push!(my_duals, var)
end
my_duals # a vector of anonymous variables
