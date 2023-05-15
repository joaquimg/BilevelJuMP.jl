# Troubleshooting

* Cbc has known bugs in its SOS1 constraints, so `BilevelJuMP.SOS1Mode` might
not work properly with Cbc.

* For anonymous variables with `DualOf` use:
```julia
@variable(Upper(model, variable_type = DualOf(my_lower_constraint)))
```

* Nonconvex/nonconcave/nonpsd objective/constraint error in a MIP solver.
If you are using
[`Gurobi`](https://github.com/jump-dev/Gurobi.jl#using-gurobi-v90-and-you-got-an-error-like-q-not-psd)
use:
```julia
model = BilevelModel(Gurobi.Optimizer, mode = BilevelJuMP.SOS1Mode()) #or other mode
set_optimizer_attribute(model, "NonConvex", 2)
```
