# BilevelJuMP.jl

A bilevel optimization extension of the [JuMP](https://github.com/JuliaOpt/JuMP.jl) package.

| **Build Status** |
|:----------------:|
| [![Build Status][build-img]][build-url] [![Codecov branch][codecov-img]][codecov-url] |


[build-img]: https://travis-ci.org/joaquimg/BilevelJuMP.jl.svg?branch=master
[build-url]: https://travis-ci.org/joaquimg/BilevelJuMP.jl
[codecov-img]: http://codecov.io/github/joaquimg/BilevelJuMP.jl/coverage.svg?branch=master
[codecov-url]: http://codecov.io/github/joaquimg/BilevelJuMP.jl?branch=master

## Introduction

BilevelJuMP is a package for modeling and solving bilevel optimization problems in Julia. As an extension of the JuMP package, BilevelJuMP allows users to employ the usual JuMP syntax with minor modifications to describe the problem and query solutions.

BilevelJuMP is built on top of [MathOptInterface](https://github.com/JuliaOpt/MathOptInterface.jl) and makes strong use of its features to reformulate the problem as a single level problem and solve it with available MIP, NLP, and other solvers.

The currently available methods are based on re-writing the problem using the KKT conditions of the lower level. For that we make strong use of [Dualization.jl](https://github.com/JuliaOpt/Dualization.jl)

## Example

```julia
using JuMP, BilevelJuMP, Xpress

model = BilevelModel()

@variable(Lower(model), x)
@variable(Upper(model), y)

@objective(Upper(model), Min, 3x + y)
@constraints(Upper(model), begin
    x <= 5
    y <= 8
    y >= 0
end)

@objective(Lower(model), Min, -x)
@constraints(Lower(model), begin
     x +  y <= 8
    4x +  y >= 8
    2x +  y <= 13
    2x - 7y <= 0
end)

optimize!(model, Xpress.Optimizer(), BilevelJuMP.SOS1Mode())

objective_value(model) # = 3 * (3.5 * 8/15) + 8/15
value(x) # = 3.5 * 8/15
value(y) # = 8/15
```

The option `BilevelJuMP.SOS1Mode()` indicates that the solution method used will be a KKT reformulation emplying SOS1 to model complementarity constraints and solve the problem with MIP solvers (Cbc, Xpress, Gurobi, CPLEX).

Another option is `BilevelJuMP.ProductMode()` that reformulates the complementarity constraints as products so that the problem can be solved by NLP (Ipopt, KNITRO) solvers or even MIP solver with the aid of binary expansions (see [QuadraticToBinary.jl](https://github.com/joaquimg/QuadraticToBinary.jl)).
