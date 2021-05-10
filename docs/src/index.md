# BilevelJuMP.jl Documentation

## Introduction

BilevelJuMP is a package for modeling and solving bilevel optimization problems in Julia. As an extension of the JuMP package, BilevelJuMP allows users to employ the usual JuMP syntax with minor modifications to describe the problem and query solutions.

BilevelJuMP is built on top of [MathOptInterface](https://github.com/JuMP-dev/MathOptInterface.jl) and makes strong use of its features to reformulate the problem as a single level problem and solve it with available MIP, NLP, and other solvers.

The currently available methods are based on re-writing the problem using the KKT conditions of the lower level. For that we make strong use of [Dualization.jl](https://github.com/JuMP-dev/Dualization.jl)