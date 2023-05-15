# Bilevel Optimization

Bilevel optimization is a vast discipline with a long (50+ years) history.
We will not attempt to present a bilevel optimization introduction here.
Instead, we point the reader to the excelent text:

[A Gentle and Incomplete Introduction to Bilevel Optimization](https://optimization-online.org/wp-content/uploads/2021/06/8450-1.pdf)

by Yasmine Beck and Martin Schmidt.

The interested reader can find more information in books:

* [Dempe 2002](https://doi.org/10.1007/b101970)
* [Bard 2013](https://doi.org/10.1007/978-1-4757-2836-1)
* [Dempe et al. 2015](https://doi.org/10.1007/978-3-662-45827-3)

and in other reviews:

* [Vicente and Calamai 1994](https://doi.org/10.1007/BF01096458)
* [Colson et al. 2007](https://doi.org/10.1007/s10479-007-0176-2)
* [Kalashnikov et al. 2015](https://doi.org/10.1155/2015/310301)
* [Dempe (2018)](https://optimization-online.org/wp-content/uploads/2018/08/6773.pdf)

## Bilevel Optimization in BilevelJuMP

In BileveJuMP focus on the following bilevel problem form:

```math
\begin{aligned}
    &\min_{x, \textbf{y}, z} && f_0(x, \textbf{y}, z) \\
    &\textit{s.t.} && f_i(x, \textbf{y}, z) \in \mathcal{S}_i, \quad i = 1 \ldots k, \\
        &&& x(z), \textbf{y}(z) \in
     \begin{aligned}[t]
        &\arg\min_{x, \textbf{y}} && \frac{1}{2}{[x, z]}^{\top} Q {[x, z]} + {a_0}^{\top} x+ {d_0}^{\top} z + b_0\\
            &\textit{s.t.} && A_i x + D_i z + b_i \in \mathcal{C}_i \quad : \ y_i \quad, \quad i = 1 \ldots m,
     \end{aligned}
\end{aligned}
```

where ``z \in {\mathbb{R}^l}`` and ``x \in {\mathbb{R}^n}`` are, respectively, from upper and lower-level primal decision variables. Vectors of lower level dual decision variables are represented individually, by ``y_1 \in \mathbb{R}^{p_1}, \ldots, y_m \in \mathbb{R}^{p_m}``, or jointly, by the ``m``-tuple ``\textbf{y} = (y_1, \ldots, y_m)``.
The square brackets ``[\cdot, \cdot]`` represent the stacking of two vectors or scalars. Thus,
``[x,z]`` is a ``(n+l)``--vector with the elements of ``x`` and ``z`` stacked. The numbers of constraints in the upper and lower problems are given by ``k`` and ``m``, respectively. ``Q``, ``a_i``, ``d_i``, ``b_i``, ``A_i``, ``D_i`` are matrices (upper case) and vectors (lower case) of constants of the lower level problem and ``\mathcal{C}_i \subset \mathbb{R}^{p_i}`` are convex conic sets. The functions ``f_i`` can be linear, quadratic, or non-linear. The sets ``\mathcal{S}_i\in \mathbb{R}^{q_i}`` can be convex cones, as in the lower level, but can also represent other sets, such as the sets of integers or binary variables.
We use the "function-in-set" notation following the MOI definition of mathematical optimization problems.
As in traditional bilevel programming, ``z`` is decided in the upper level and passed to the lower level as a parameter and ``x`` might be seen as an upper-level variable constrained to be an optimal solution of the lower level.
Also, we only consider optimistic bilevel problems. In short, the solution of the lower level will be the one that optimizes the upper level in case of degeneracy.

For more information see our paper:

```
@article{diasgarcia2022bileveljump,
    title={{BilevelJuMP. jl}: {M}odeling and solving bilevel optimization in {J}ulia},
    author={{Dias Garcia}, Joaquim and Bodin, Guilherme and Street, Alexandre},
    journal={arXiv preprint arXiv:2205.02307},
    year={2022}
}
```

Here is the [pdf](https://arxiv.org/pdf/2205.02307.pdf).