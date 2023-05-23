# BilevelJuMP.jl

A bilevel optimization extension of the [JuMP](https://github.com/JuMP-dev/JuMP.jl) package.

| **Build Status** |
|:----------------:|
| [![Build Status][build-img]][build-url] [![Codecov branch][codecov-img]][codecov-url] [![Docs dev][docs-dev-img]][docs-dev-url] [![Docs stable][docs-stable-img]][docs-stable-url]|


[build-img]: https://github.com/joaquimg/BilevelJuMP.jl/workflows/CI/badge.svg?branch=master
[build-url]: https://github.com/joaquimg/BilevelJuMP.jl/actions?query=workflow%3ACI
[codecov-img]: http://codecov.io/github/joaquimg/BilevelJuMP.jl/coverage.svg?branch=master
[codecov-url]: http://codecov.io/github/joaquimg/BilevelJuMP.jl?branch=master
[docs-dev-img]: https://img.shields.io/badge/docs-latest-blue.svg
[docs-dev-url]: https://joaquimg.github.io/BilevelJuMP.jl/dev/
[docs-stable-img]: https://img.shields.io/badge/docs-stable-blue.svg
[docs-stable-url]: https://joaquimg.github.io/BilevelJuMP.jl/stable/

[BilevelJuMP.jl](https://github.com/joaquimg/BilevelJuMP.jl) is a JuMP extension for
modeling and solving bilevel optimization problems.

## License

`BilevelJuMP.jl` is licensed under the [MIT license](https://github.com/joaquimg/BilevelJuMP.jl/blob/master/LICENSE).

## Documentation

You can find the documentation at
[https://joaquimg.github.io/BilevelJuMP.jl/stable/](https://joaquimg.github.io/BilevelJuMP.jl/stable).

## Help

If you need help, please [open a GitHub issue](https://github.com/joaquimg/BilevelJuMP.jl/issues/new).

## Example

```julia
using BilevelJuMP, HiGHS

model = BilevelModel(
    HiGHS.Optimizer,
    mode = BilevelJuMP.FortunyAmatMcCarlMode(primal_big_M = 100, dual_big_M = 100)
)

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

optimize!(model)

objective_value(model) # = 3 * (3.5 * 8/15) + 8/15 # = 6.13...
value(x) # = 3.5 * 8/15 # = 1.86...
value(y) # = 8/15 # = 0.53...
```

