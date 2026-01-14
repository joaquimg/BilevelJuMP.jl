# # Foundations of Bilevel Programming: Chapter 3.7, Page 59
#
# This example is from the book _Foundations of Bilevel Programming_ by Stephan
# Dempe, Chapter 3.7, Page 59. [url](https://www.springer.com/gp/book/9781402006319)

# Model of the problem

# First level
# ```math
# \min \sum_{i\in I} x_i - z,\\
# \notag s.t.\\
# y^a_i\geq 0, \forall i \in I,\\
# y^a_i\leq 1, \forall i \in I,\\
# y^b_i\geq 0, \forall i \in I,\\
# y^b_i\leq 1, \forall i \in I,\\
# y^a_i + y^b_i = 1, \forall i \in I,\\
# z\geq 0,\\
# z\leq 1,\\
# y^a_1 + y^a_2 + y^a_3 \geq z,\\
# -y^b_1 - y^b_4 + y^a_3 \geq z,\\
# y^b_7 - y^b_6 + y^a_4 \geq z,\\
# y^a_5 + y^a_6 + y^a_7 \geq z.\\
# ```
# Second level
# ```math
# \min -\sum_{i\in I}x_i,\\
# \notag s.t.\\
# \sum x_i \geq 0,\\
# x_i \leq y^a_i, \forall i\in I,\\
# x_i \leq y^b_i, \forall i\in I,\\
# I = \{1,...,7\}
# ```

using BilevelJuMP
using Ipopt
using Test #src

model = BilevelModel(Ipopt.Optimizer; mode = BilevelJuMP.ProductMode(1e-9))

# Global variables

I = 7 # maximum literals
clauses = [[1, 2, 3], [-1, -4, 3], [7, -6, 4], [5, 6, 7]]

# First we need to create all of the variables in the upper and lower problems:

# Upper level variables
@variable(Upper(model), ya[i = 1:I])
@variable(Upper(model), yb[i = 1:I])
@variable(Upper(model), z)

#Lower level variables
@variable(Lower(model), x[i = 1:I])

# Then we can add the objective and constraints of the upper problem:

# Upper level objecive function
@objective(Upper(model), Min, sum(x[i] for i in 1:I) - z)

# Upper level constraints
@constraint(Upper(model), ca, z <= 1)
@constraint(Upper(model), cb, z >= 0)
@constraint(Upper(model), c1[i = 1:I], ya[i] >= 0)
@constraint(Upper(model), c2[i = 1:I], ya[i] <= 1)
@constraint(Upper(model), c3[i = 1:I], yb[i] >= 0)
@constraint(Upper(model), c4[i = 1:I], yb[i] <= 1)
@constraint(Upper(model), c5[i = 1:I], ya[i] + yb[i] == 1)

# for c in clauses
@constraint(
    Upper(model),
    cc[k in eachindex(clauses)],
    sum(i > 0 ? ya[i] : yb[-i] for i in clauses[k]) >= z
)

# Followed by the objective and constraints of the lower problem:

# Lower objective function
@objective(Lower(model), Min, -sum(x[i] for i in 1:I))

# Lower constraints
@constraint(Lower(model), b1[i = 1:I], x[i] >= 0)
@constraint(Lower(model), b2[i = 1:I], x[i] <= ya[i])
@constraint(Lower(model), b3[i = 1:I], x[i] <= yb[i])

# Initial Starting conditions

JuMP.set_start_value.(x, 0)
JuMP.set_start_value.(ya, 1)
JuMP.set_start_value.(yb, 0)
JuMP.set_start_value(z, 1)
for i in 1:I
    JuMP.set_dual_start_value.(b1, 0)
    JuMP.set_dual_start_value.(b2, 0)
    JuMP.set_dual_start_value.(b3, -1)
end

# Now we can solve the problem and verify the solution again that reported by
# Dempe.

optimize!(model)

# 
primal_status(model)

#

termination_status(model)

# Results

objective_value(model)

#

value.(x)

#

value.(ya)

#

value.(yb)

#

value(z)


# Auto testing #src
atol = 1e-4 #src
@test objective_value(model) ≈ -1 atol = atol #src
@test value.(x) ≈ zeros(I) atol = atol #src
@test value.(ya) ≈ ones(I) atol = atol #src
@test value.(yb) ≈ zeros(I) atol = atol #src
@test value(z) ≈ 1 atol = atol #src

