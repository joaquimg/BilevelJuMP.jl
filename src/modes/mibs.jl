# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

"""
    MibSMode(mibs_call; verbose = false, check_integrality = true)

Solve the bilevel problem with [MibS](https://github.com/coin-or/MibS), an
external mixed integer bilevel solver.

Unlike the other modes, MibS does not reformulate the problem into a single
optimization problem that is handed to a MathOptInterface solver. It is a
standalone executable that reads the problem from a file, so there is no
optimizer to attach and `set_optimizer` must not be called.

* `mibs_call` is the MibS executable. It is obtained from the `MibS_jll`
  package, which is deliberately **not** a dependency of BilevelJuMP: install
  and load it yourself, then pass `MibS_jll.mibs`.

* `verbose` prints the MibS log.

* `check_integrality` errors if the model has a continuous variable. MibS may
  run forever instead of reporting an error on such a model, so this check is
  on by default. Set it to `false` to attempt the solve anyway.

MibS requires both levels to be linear and, in practice, every variable to be
integer. Models it cannot represent are rejected with an error by `optimize!`.

Results are queried with the usual JuMP functions: `termination_status`,
`primal_status`, `objective_value`, and `value`. Dual solutions are not
available, because MibS never forms the dual of the lower level.

## Example

```julia
using BilevelJuMP, MibS_jll

model = BilevelModel()
BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(MibS_jll.mibs))

@variable(Upper(model), x, Int)
@variable(Lower(model), y, Int)

@objective(Upper(model), Min, -3x - 7y)
@constraint(Upper(model), u1, x <= 10)

@objective(Lower(model), Min, y)
@constraint(Lower(model), l1, 2x - y <= 7)

optimize!(model)

termination_status(model)
objective_value(model)
value(x)
value(y)
```
"""
mutable struct MibSMode{T} <: AbstractBilevelSolverMode{T}
    mibs_call::Any
    verbose::Bool
    check_integrality::Bool
    function MibSMode(
        mibs_call = nothing;
        verbose::Bool = false,
        check_integrality::Bool = true,
    )
        return new{Float64}(mibs_call, verbose, check_integrality)
    end
end
