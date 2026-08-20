# # Conic Bilevel and Mixed Mode

# Here we present a simple bilevel program with a conic lower level model
# described in example 3.3 from
# [Chi, et al. 2014](https://journalofinequalitiesandapplications.springeropen.com/articles/10.1186/1029-242X-2014-168).

# ```math
# \begin{align}
#     &\max_{x \in \mathbb{R}} \quad x + 3y_1 \\
#     &\textit{s.t.} \quad 2 \leq x \leq 6\\
#     & \hspace{28pt} y(x) \in \arg\min_{y\in {\mathbb{R}^3}} -y_1\\
#             & \hspace{58pt} \textit{s.t.} \quad x + y_1 \leq 8 \\
#             & \hspace{76pt} \quad x + 4y_1 \geq 8 \\
#             & \hspace{76pt}  \quad x + 2y_1 \leq 12 \\
#             & \hspace{76pt}  \quad y \in {SOC}_3 \label{eq-soc}
# \end{align}
# ```

# In this problem most of the constraints are regular linear constraints
# while the last one is a second order cone constraint.
# Such constraint ensures that the vector $y$ belongs to a second order
# cone of dimension $3$, that is: $y_1 \geq \sqrt{y_2^2 + y_3^2}$.

# This problem can be encoded using regular JuMP syntax for conic programs:

using BilevelJuMP
model = BilevelModel()
@variable(Upper(model), x)
@variable(Lower(model), y[i=1:3])
@objective(Upper(model), Min, x + 3y[1])
@constraint(Upper(model), x >= 2)
@constraint(Upper(model), x <= 6)
@objective(Lower(model), Min, - y[1])
@constraint(Lower(model), con1, x +  y[1] <=  8)
@constraint(Lower(model), con2, x + 4y[1] >=  8)
@constraint(Lower(model), con3, x + 2y[1] <= 12)
@constraint(Lower(model), con4, y in SecondOrderCone())

# ## NLP solution and start values

# We can set, for instance, the product reformulation and selected Ipopt
# as a solver. As Ipopt does not have native support for second order cones,
# we use the non-default MOI bridge `SOCtoNonConvexQuad` to convert
# second order cones into quadratic constraints.

using Ipopt
BilevelJuMP.set_mode(model, BilevelJuMP.ProductMode(1e-5))
set_optimizer(model,
    ()->MOI.Bridges.Constraint.SOCtoNonConvexQuad{Float64}(Ipopt.Optimizer()))
optimize!(model)

# This problem is very simple, but more complex models might require more  #src
# information such as starting points that can be passed on variable creation #src
# with standard JuMP syntax, for instance: #src

# @variable(Upper(model), x, start = 6) #src

# The user could also use the alternative JuMP syntax:

set_start_value(x, 6)
set_dual_start_value(con2, 0)

# ## MIP solution and mixed mode

# Alternatively, we could have used a Mixed Integer Second Order Cone Program
# (MISOCP) solver together with binary expansions. Complementarity of conic
# constraints is more difficult to handle because they require a sum of products
# that cannot be reformulated with other methods. Therefore, we rely on product
# reformulation for conic constraints. However, we can use other reformulations
# like indicator constraints for the non-conic constraints.
# Mixing the two of them can be done with Mixed Mode.

# The following code describes how to solve the problem with a MISOCP based solver.

# ```julia
# using Xpress
# using QuadraticToBinary
# set_optimizer(model,
#     ()->QuadraticToBinary.Optimizer{Float64}(Xpress.Optimizer(),lb=-10,ub=10))
# BilevelJuMP.set_mode(model, 
#     BilevelJuMP.MixedMode(default = BilevelJuMP.IndicatorMode()))
# BilevelJuMP.set_mode(con4, BilevelJuMP.ProductMode(1e-5))
# optimize!(model)
# ```

# !!! info
#     This code was not executed because Xpress requires a commercial license.
#     Other solvers supporting MISOCP could be used such as Gurobi and CPLEX.

# We set the reformulation method as Mixed Mode and selected Indicator
# constraints to be the default for the case in which we do not explicitly
# specify the reformulation.
# Then we set product mode for the second order cone reformulation.

# Binary expansions require bounded
# variables, hence the `QuadraticToBinary` meta-solver accepts fallback
# to upper and lower bounds (\texttt{ub} and \texttt{lb}),
# used for variables with no explicit bounds.