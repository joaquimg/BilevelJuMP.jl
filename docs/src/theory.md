
# Theory of Bi-level Optimization
## Introduction

Standard single-level optimization problems consist of a single decision maker who controls a set of decision variables to optimize its goal. However, in many real-world problems, several decision makers with conflicting goals interact and each individual decision impacts the decisions and outcomes of the others. Such interdependencies cannot be captured with the standard setting of a single decision maker and other, more adequate, optimization problems are required [[1]](#1).


## References
<a id="1">[1]</a> 
Dijkstra, E. W. (1968). 
Go to statement considered harmful. 
Communications of the ACM, 11(3), 147-148.
=======



```math
\begin{align}
& \min_{x \in \mathbb{R}^n} & a_0^T x + b_0
\\
& \;\;\text{s.t.} & A_i x + b_i & \in \mathcal{C}_i & i = 1 \ldots m
\end{align}
```

