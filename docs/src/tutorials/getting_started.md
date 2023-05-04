```@meta
EditURL = "<unknown>/src/tutorials/getting_started.jl"
```

# Getting started with BilevelJuMP

This is a quick introduction to modeling and solving bilevel optimization
with BilevelJuMP.

If you are new to Julia, start with the
[Getting started with Julia](https://jump.dev/JuMP.jl/stable/tutorials/getting_started/getting_started_with_julia/)
from the JuMP documentation.

If you are new to JuMP, start with the
[Getting started with JuMP](https://jump.dev/JuMP.jl/stable/tutorials/getting_started/getting_started_with_JuMP/)
from the JuMP documentation.

## Installation

BilevelJuMP is a JuMP extension that be installed
by using the built-in package manager.

```julia
import Pkg
Pkg.add("BilevelJuMP")
```

That is all you need to model a bilevel optimization problem, but we want to
also *solve* the problems. Therefore we need a solver, one such solver is
`HiGHS.Optimizer`, which is provided by the
[HiGHS.jl](https://github.com/jump-dev/HiGHS.jl) package.

```julia
import Pkg
Pkg.add("HiGHS")
```

## A first example

We will solve the following bilevel optimization problem using BilevelJuMP
and HiGHS. First we take a look in the entire code then we go through it
step-by-step.

Here is the example from Dempe (2002), Chapter 3.2, Page 25:

```math
\begin{align*}
    &\min_{x, y} && 3x + y \\
    &\st && x \leq 5 \\
    &    && y \leq 8 \\
    &    && y \geq 0 \\
    &    && x(y) \in
     \begin{aligned}[t]
        &\argmin_{x} && -x\\
            &\st && x + y \leq 8\\
            &    && 4x + y \geq 8\\
            &    && 2x + y \leq 13\\
            &    && 2x - 7y \leq 0
     \end{aligned}
\end{align*}
```

Here is the complete code to model, solve and query results from the example:

```@example getting_started
using BilevelJuMP
using HiGHS

model = BilevelModel(
    HiGHS.Optimizer,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100))

@variable(Lower(model), x)

@variable(Upper(model), y)

@objective(Upper(model), Min, 3x + y)

@constraint(Upper(model), u1, x <= 5)
@constraint(Upper(model), u2, y <= 8)
@constraint(Upper(model), u3, y >= 0)

@objective(Lower(model), Min, -x)

@constraint(Lower(model), l1, x +  y <= 8)
@constraint(Lower(model), l2, 4x +  y >= 8)
@constraint(Lower(model), l3, 2x +  y <= 13)
@constraint(Lower(model), l4, 2x - 7y <= 0)

print(model)

optimize!(model)

termination_status(model)

primal_status(model)

dual_status(Lower(model))

dual_status(Upper(model))

objective_value(model)

objective_value(Lower(model))

objective_value(Upper(model))

value(x)

value(y)

dual(l1)

dual(l2)
```

## Step-by-step

Once installed, BilevelJuMP can be loaded into julia:

```@example getting_started
using BilevelJuMP
```

Note that JuMP comes inside BilevelJuMP, and does not need to be installed
separately. Once loaded, all JuMP functions are exported along with the
BilevelJuMP additional functions.

We include a solver, in this case HiGHS:

```@example getting_started
using HiGHS
```

Just like regular JuMP has the `Model` function to initialize an optimization
problem, BilevelJuMP has the `BilevelModel` function that takes a solver as
a first positional argument, in this case `HiGHS.Optimizer` and a `mode`
keyword argument that selects
a bilevel solution method, in this case, `BilevelJuMP.FortunyAmatMcCarlMode`.
Note that `BilevelJuMP.FortunyAmatMcCarlMode` takes two optional keyword
arguments: `primal_big_M` and `dual_big_M` which have to be larger than the
value of all primal and dual variavle sof the lower level respectively to
guarantee that the solution is no eliminated.

```@example getting_started
model = BilevelModel(
    HiGHS.Optimizer,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100))
```

For more on `mode`s and solutions methods, see XXX.

We can proceed, as usual in JuMP models and incrementally build our bilevel
problem. We use the same macros as JuMP.

Variables are modeled using the `@variable` macro, but in bilevel problems
we must define which level the variable is decided, then we use the
`Upper` and `Lower` constructors to direct variable to the proper levels:

```@example getting_started
@variable(Lower(model), x)

@variable(Upper(model), y)
```

The same goes for objective that are modeled with the `@objective` macro:

```@example getting_started
@objective(Upper(model), Min, 3x + y)
```

and constraints that are modeled with the `@objective` macro:

```@example getting_started
@constraint(Upper(model), u1, x <= 5)
@constraint(Upper(model), u2, y <= 8)
@constraint(Upper(model), u3, y >= 0)
```

repeat for the lower level:

```@example getting_started
@objective(Lower(model), Min, -x)

@constraint(Lower(model), l1, x +  y <= 8)
@constraint(Lower(model), l2, 4x +  y >= 8)
@constraint(Lower(model), l3, 2x +  y <= 13)
@constraint(Lower(model), l4, 2x - 7y <= 0)
```

display the model

```@example getting_started
print(model)
```

solve the bilevel problem, which will combine a `mode` (in this case
`FortunyAmatMcCarlMode`) and a solver (in this case `HiGHS`):

```@example getting_started
optimize!(model)
```

check the `termination_status` to understand why the solver stopped:

```@example getting_started
termination_status(model)
```

check the `primal_status` to check if there is a feasible solution available:

```@example getting_started
primal_status(model)
```

check the `dual_status` to check if there is a dual solution available for the
lower level:

```@example getting_started
dual_status(Lower(model))
```

do the same for the upper level:

```@example getting_started
dual_status(Upper(model))
```

!!! info
    Most method will not support upper level duals.

!!! info
    JuMP's `dual_status` is not available to `BilevelModel`'s although
    you can query `dual_status` of each level.

Query the objecive value of the bilevel model

```@example getting_started
objective_value(model)
```

Query the objective value of the lower level and the upper level

```@example getting_started
objective_value(Lower(model))

objective_value(Upper(model))
```

Obtain primal solutions:

```@example getting_started
value(x)

value(y)
```

and dual solutions:

```@example getting_started
dual(l1)

dual(l2)
```

## Model basics

We created a BilevelModel passing the optimizer and mode and initialization:

```@example getting_started
model = BilevelModel(
    HiGHS.Optimizer,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100))
```

We could do piece by piece

```@example getting_started
model = BilevelModel()

set_optimizer(model, HiGHS.Optimizer)

BilevelJuMP.set_mode(model,
    BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100))
```

!!! warning
    Both `BilevelModel` and `set_optimizer` take a optimizer *constructor*,
    in this case `HiGHS.Optimizer`. Note that `HiGHS.Optimizer()` returns an
    instance of the `HiGHS.Optimizer`. Hence, and alternative way to pass this
    solver would be: `set_optimizer(model, () -> HiGHS.Optimizer())`.

    `() -> HiGHS.Optimizer()` is a an anonymous function that returns an
    instance of the `HiGHS.Optimizer`.

!!! info
    Ther is no equivalent of JuMP's `direct_model` in BilevelJuMP.

## Solver options

it is also possible to pass optimizers with attributes:

```@example getting_started
model = BilevelModel(
    optimizer_with_attributes(HiGHS.Optimizer, "output_flag" => false))
```

or set such attributes separately:

```@example getting_started
model = BilevelModel(HiGHS.Optimizer)

set_attribute(model, "output_flag", false)

get_attribute(model, "output_flag")
```

!!! info
    [View this file on Github](<unknown>/src/tutorials/getting_started.jl).

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

