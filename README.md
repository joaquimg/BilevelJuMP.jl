# BilevelJuMP.jl

[![Build Status](https://github.com/joaquimg/BilevelJuMP.jl/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/joaquimg/BilevelJuMP.jl/actions?query=workflow%3ACI)
[![Codecov branch](http://codecov.io/github/joaquimg/BilevelJuMP.jl/coverage.svg?branch=master)](http://codecov.io/github/joaquimg/BilevelJuMP.jl?branch=master)
[![Docs dev](https://img.shields.io/badge/docs-latest-blue.svg)](https://joaquimg.github.io/BilevelJuMP.jl/dev/)
[![Docs stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://joaquimg.github.io/BilevelJuMP.jl/stable/)

[BilevelJuMP.jl](https://github.com/joaquimg/BilevelJuMP.jl) is a
[JuMP](https://github.com/JuMP-dev/JuMP.jl) extension for
modeling and solving bilevel optimization problems.

## License

`BilevelJuMP.jl` is licensed under the [MIT license](https://github.com/joaquimg/BilevelJuMP.jl/blob/master/LICENSE).

## Documentation

You can find the documentation at
[https://joaquimg.github.io/BilevelJuMP.jl/stable/](https://joaquimg.github.io/BilevelJuMP.jl/stable).

## Help

If you need help, please [open a GitHub issue](https://github.com/joaquimg/BilevelJuMP.jl/issues/new).

## Example

### Install

```julia
import Pkg
Pkg.add("BilevelJuMP")
Pkg.add("HiGHS")
```

### Run

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

## Citing BilevelJuMP

If you use BilevelJuMP.jl, we ask that you please cite the following [paper](https://pubsonline.informs.org/doi/10.1287/ijoc.2022.0135):

```bibtex
@article{diasgarcia2023bileveljump,
    title = {{BilevelJuMP.jl}: {M}odeling and {S}olving {B}ilevel {O}ptimization {P}roblems in {J}ulia},
    author = {{Dias Garcia}, Joaquim and Bodin, Guilherme and Street, Alexandre},
    journal = {INFORMS Journal on Computing},
    doi = {https://doi.org/10.1287/ijoc.2022.0135},
    pages = {1-9},
    year = {2023}
}
```

Here is an earlier [preprint](https://arxiv.org/pdf/2205.02307.pdf).
