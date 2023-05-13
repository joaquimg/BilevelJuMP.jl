# BilevelJuMP.jl Documentation

## Introduction

BilevelJuMP is a package for modeling and solving bilevel optimization problems
in Julia.

As an extension of the [JuMP](https://jump.dev/) modeling language, BilevelJuMP
allows users to employ the usual JuMP syntax with minor modifications to
describe the problem and query solutions.

Many modeling features are available in BilevelJuMP, some of which are unique
while others are also not widely available. The main features supported are:

- Arbitrary JuMP models in the upper level (NLP, Conic, MIP)
- Conic constraints and quadratic objectives in the lower-level
- Dual variables of the lower level in the upper level
- MPEC reformulations with MIP or NLP solvers
- MixedMode MPEC reformulation: select the best reformulation for each
constraint separately

The currently available methods are based on re-writing the problem using the
KKT conditions of the lower level. For that we make strong use of
[Dualization.jl](https://github.com/JuMP-dev/Dualization.jl)

## Installation

Install BilevelJuMP as follows:

```julia
julia> import Pkg

julia> Pkg.add("BilevelJuMP")
```

## Getting started

- Learn the basics of
[JuMP](https://jump.dev/JuMP.jl/stable/tutorials/getting_started/getting_started_with_JuMP/) and
[Julia](https://jump.dev/JuMP.jl/stable/tutorials/getting_started/getting_started_with_julia/)
in the
[JuMP documentation](https://jump.dev/JuMP.jl/stable/)
- Follow the tutorials in this manual

If you need help, please open a GitHub issue.

## License

BilevelJuMP.jl is licensed under the
[MIT License](https://github.com/joaquimg/BilevelJuMP.jl/blob/master/LICENSE).

## Citing `BilevelJuMP.jl`

If you use BilevelJuMP.jl, we ask that you please cite the following:

```
@article{diasgarcia2022bileveljump,
    title={{BilevelJuMP.jl}: {M}odeling and solving bilevel optimization in {J}ulia},
    author={{Dias Garcia}, Joaquim and Bodin, Guilherme and Street, Alexandre},
    journal={arXiv preprint arXiv:2205.02307},
    year={2022}
}
```

Here is the [pdf](https://arxiv.org/pdf/2205.02307.pdf).
