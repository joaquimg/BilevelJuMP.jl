# # SOCBLP from Chi et al.: Example 3.1
#
# This example is from The models of bilevel programming with lower level
# second-order cone programs
# ([url](https://journalofinequalitiesandapplications.springeropen.com/articles/10.1186/1029-242X-2014-168) or [url](https://core.ac.uk/download/pdf/81261904.pdf))
# SOCBLP stands for bilevel programming problem with lower level second-order cone program
# Bold point(s): Using second-order cone in the lower level problem 

# Model of the problem
# First level
# ```math
# \min x + 3(y_1-y_2),\\
# \notag s.t.\\
# 2 \leq x \leq 6,\\
# ```
# Second level
# ```math
# \min - y_1 + y_2,\\
# \notag s.t.\\
# x + y_1 - y_2 \leq  8,\\
# x + 4(y_1 - y_2) \geq  8,\\
# x + 2(y_1 - y_2) \leq  12,\\
# y_1 \geq 0,\\
# y_2 \geq 0,\\
# y \in K^2\\
# ```

# Here, ``K^2`` is second order cone and it represents:
# ```math
# y \in \{y=(y_1,y_2)\in \mathbb{R}\times \mathbb{R}:y_1 \geq ||y_2||_2\}\\
# ```

using BilevelJuMP
using Ipopt
using Test #src

model = BilevelModel(
    () -> MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(Ipopt.Optimizer());
    mode = BilevelJuMP.ProductMode(1e-9),
)

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), x)

#Lower level variables
@variable(Lower(model), y[i = 1:2])

# Then we can add the objective and constraints of the upper problem:

# Upper level objecive function
@objective(Upper(model), Min, x + 3(y[1] - y[2]))

# Upper level constraints
@constraint(Upper(model), x >= 2)
@constraint(Upper(model), x <= 6)

# Followed by the objective and constraints of the lower problem:

# Lower objective function
@objective(Lower(model), Min, -(y[1] - y[2]))

# Lower constraints
@constraint(Lower(model), lb_y_1, y[1] >= 0)
@constraint(Lower(model), lb_y_2, y[2] >= 0)
@constraint(Lower(model), con1, x + (y[1] - y[2]) <= 8)
@constraint(Lower(model), con2, x + 4(y[1] - y[2]) >= 8)
@constraint(Lower(model), con3, x + 2(y[1] - y[2]) <= 12)
@constraint(Lower(model), soc_lw, y in SecondOrderCone())

# Defining bounds
BilevelJuMP.set_dual_upper_bound_hint(soc_lw, +[5.0, 5.0])
BilevelJuMP.set_dual_lower_bound_hint(soc_lw, -[5.0, 5.0])

# require lower bounds
for con in [con1, con3]
    BilevelJuMP.set_dual_lower_bound_hint(con, -15)
end
# require upper bounds
for con in [lb_y_1, lb_y_2, con2]
    BilevelJuMP.set_dual_upper_bound_hint(con, +15)
end
# bounds defined in the upper level are not dualized
for i in 1:2
    @constraint(Upper(model), y[i] in MOI.LessThan(+5.0))
    @constraint(Upper(model), y[i] in MOI.GreaterThan(-5.0))
end

# Now we can solve the problem and verify the solution again that reported by

optimize!(model)

primal_status(model)

termination_status(model)

objective_value(model)

value.(y)

@test objective_value(model) ≈ 12 atol = 1e-1 #src
@test value(x) ≈ 6 atol = 1e-3 #src
@test value(y[2]) >= 0 - 1e-3 #src
@test value(y[1]) - value(y[2]) ≈ 2 atol = 1e-3 #src
