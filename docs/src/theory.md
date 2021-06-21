
# Theory of Bi-level Optimization
## Introduction
Primarily, research in mathematical optimization has been focused on **idealized** models. In the definition of the ideal model, we have only a single decision-maker (DM) facing the problem of making a single set of decisions at a single point in time and under perfect knowledge. Over the past decades, different approaches have been well developed with high-quality implementation widely available in off-the-shelf software for these mathematical problems. The general structure of an ideal problems consists of an objective function subject to some conditions or, better to say, some constraints like

```math
	\notag\begin{align}
	&\textbf{(GO)}\\
	&\min_{x\in X} f(x),
	\end{align}
```

Where ``X\subset\mathbb{R}^n`` is the feasible region defined by a set of constraints and ``f:\mathbb{R}^n\rightarrow\mathbb{R}`` is the objective function. 

Despite its widespread usage of ideal models, the property of many real-world problems does not align with this category. Most real-world applications involve multiple DMs, multiple competing objectives, and/or decisions made at multiple points in time. We are not able to capture the complexity of these problems with the conventional definition of ideal models. Therefore new categories have been introduced under the title of multistage and multilevel optimization to address these complexities.

The multistage optimization problem is mainly about a single MD whose decisions are taken over multiple time periods with an objective that encapsulates the impact of future stages at the current time. On the other hand, in multilevel optimization, we are dealing with multiple MDs who make decisions simultaneously or in a subsequent hierarchical structure. In this package, we are mainly focused on problems so-called Stackelberg, in which there are two MDs with hierarchical structure decision; any decision taken by the upper-level authority (leader) to optimize their goals is affected by the response of lower-level entities (follower), who will seek to optimize their outcomes. These hierarchical optimization problems with two levels are commonly called bilevel problems. Ankur Sinha et al. provide an excellent representation for this category of problems. 

``` @raw html
<figure align = "center">
 <img src="../images/Ankur.png" alt="my alt text"/ width= "500" >
 <figcaption align = "center"> hierarchical structure of bilevel problems
</figcaption>
</figure>
```

In the figure, you can see for any given upper-level decision vector, there is a corresponding (parametric) lower-level optimization problem that needs to be solved. In other words, each decision vector (``x_u``) of the leader creates an optimization problem follower. In the figure, ``x_u`` and ``x_l`` represent the leader's and follower's decision vectors, respectively. Feasible space for the upper-level problem is defined with ``(x_u, x_l^*)`` pairs where the vector ``x_l^*`` is an optimal solution to the lower-level problem. It is expected that each layer has its own objectives and constraints. In the context of bilevel optimization, the leader decides first while anticipating the rational response of the so-called Stackelberg follower, who decides second. For an overview of the vast literature on bilevel optimization, we refer to the surveys by Ankur Sinha et al. [sinha2017review](@cite), Colson et al. [colson2007overview](@cite), the books Bard [bard2013practical](@cite), Dempe [dempe2002foundations](@cite), Dempe, Kalashnikov, et al. [dempe2015bilevel](@cite), and Dempe and Zemkoho [dempe2020bilevel](@cite). 


Bi-level optimization is a step forward in modeling real worlds challenges and over the last 50 years, numerous works used bilevel optimization as a modeling tool for problems stemming from a broad spectrum of applications, such as agricultural planning (Fortuny-Amat and McCarl [fortuny1981representation](@cite), critical infrastructure defense (DeNegre [denegre2011interdiction](@cite), Caprara et al. [caprara2016bilevel](@cite), energy markets (Grimm, Schewe, et al. [grimm2019multilevel](@cite), Hu and Ralph [hu2007using](@cite), portfolio optimization (Leal et al. [leal2020portfolio](@cite), pricing (Labbé and Violin [labbe2016bilevel](@cite), or traffic planning (Migdalas [migdalas1995bilevel](@cite), to name only a few.

##Complexity of the Bilevel problems
Generally, even in the “simplest case” of using continuous variables and linear objective functions and constraints, the feasible set of bilevel problems may lead to nonconvex and disconnected regions. This means, even the “easiest” class of linear bilevel problems is considered to be NP-hard. There is a rich literature on the different approaches toward solving bi-level problems, and here we are going to mention a few. 

Candler and Norton [candler1977multi](@cite) proposed an enumerative algorithm for linear bilevel problems similar to the simplex method, but they had “no doubt others could develop more efficient algorithms”. While Bialas and Karwan [bialas1984two](@cite) proposed a similar approach, the so-called kth-best algorithm, Fortuny-Amat and McCarl [fortuny1981representation](@cite) introduced a game-changing approach for convex-quadratic bilevel problems in 1981. They replaced the follower problem with its necessary and sufficient Karush– Kuhn–Tucker (KKT) conditions to derive an equivalent single-level problem that can be further reformulated and tackled by standard mixed-integer solvers. Bard and Moore [bard1990branch](@cite), Bard [bard1988convex](@cite), Edmunds and Bard [edmunds1991algorithms](@cite), and Hansen et al. This approach is still standard for solving bilevel problems with convex follower problems today. Alternative approaches, e.g., penalty methods or descent approaches, have been proposed by Anandalingam and White [anandalingam1990solution](@cite) and Savard and Gauvin [savard1994steepest](@cite), respectively. In the 1990s, the largest instances of linear bilevel problems that have been solved consisted of 250 leader variables, 150 follower variables, and 150 follower constraints; see Hansen et al. [hansen1992new](@cite). Although cutting planes have been derived in the following years, see Audet, Haddad, et al. [audet2007disjunctive](@cite) and Audet, Savard, et al. [audet2007new](@cite), computational linear and convex bilevel optimization did not attract much attention in the 2000s and not many computational results have been reported. Similarly, Moore and Bard [candler1977multi](@cite) developed a branch-and-bound approach for bilevel problems with mixed-integer follower problems and also reported some first numerical results already in 1990. However, only very little computational progress has been reported until DeNegre and Ralphs [denegre2009branch](@cite) introduced a branch-and-cut approach for purely integer bilevel problems in 2009.


##Foundations of Bilevel Optimization
### Notation and Properties

We consider two decision makers that interact hierarchically. First, a so-called **leader** decides on its decision variab¬les ``x \in\mathbb{R}^n``. For a given leader decision ``x``, a so-called **follower** optimally reacts to this decision by solving the parametric *follower problem* 

```math
	\begin{align}
	&\textbf{(BP-F)}\\
	&\min_{y\in Y} f(x,y),\\
 & s.t.\\ 
& g(x,y)\leq 0,
	\end{align}
```
with parameter ``x``, objective function ``f(x,y): \mathbb{R}^n\times\mathbb{R}^m\rightarrow \mathbb{R}``, constraint function ``g(x,y): \mathbb{R}^n\times\mathbb{R}^m\rightarrow \mathbb{R}^l``, and a set ``Y\subseteq \mathbb{R}^m``. Given the optimal value function of the follower 


```math
	\begin{align}
	&\varphi(x):=\min_{y\in Y}\{f(x,y):g(x,y)\leq 0\},
	\end{align}
```

we denote the set of optimal follower solutions in dependence of the leader decision ``x`` by

```math
	\begin{align}
	&\Psi(x):= \{y\in Y: g(x,y)\leq 0, f(x,y)\leq &\varphi(x)\}.
	\end{align}
```

This set is sometimes also referred to as the rational reaction set. In bilevel optimization, the leader anticipates the response ``y\in\Psi(x)`` of the follower and chooses ``x\in X`` to maximize the objective function ``F(x,y): \mathbb{R}^n\times\mathbb{R}^m\rightarrow \mathbb{R}`` subject to the leader constraints ``G(x,y)\leq 0``, ``G(x,y): \mathbb{R}^n\times\mathbb{R}^m\rightarrow \mathbb{R}^k``. In general, the optimal solution of the follower problem (BP-F) is ambiguous, which means that ``\Psi(x)`` is not a singleton. In this case, we assume the optimistic or cooperative approach. Among all ambiguous follower solution ``y\in\Psi(x)``, the leader may. Select the one most favorable for it. Thus, the leader solves the optimistic bilevel problem

```math
	\begin{align}
	&\textbf{(BP)}&\\
	&\min_{x,y} F(x,y),&\\
 & s.t.&\\ 
& G(x,y)\leq 0,&x\in X\\
& y\in \arg\min_{\bar{y}}\{f(x,\bar{y}): g(x,\bar{y}),\bar{y}\in Y\}\\
	\end{align}
```
In contrast, the pessimistic approach realizes the follower solution ``y\in \Psi(x)`` that is the least favorable for the leader. We use the optimistic concept throughout this research.

In problem (BP), we call the inequality constraints of the leader and follower **linking constraints**, because they involve both sets of decision variables. Similarly, we call leader variables ``x`` (or follower variables ``y``) that appear in the follower inequality constraints (leader inequality constraints) **linking variables**. We denote the index set of linking leader variables by ``L\subseteq\{1,\dots,n\}`` and the linking leader variables by ``x_L``. In contrast to the linking constraints, the sets ``X`` and ``Y`` involves only variables from the leader or the follower, respectively. 

These sets model, e.g., simple variable bounds or integrality conditions on the decision variables. The major difficulty in this setup is that the bilevel feasible set, i.e., the feasible set of Problem (BP), is defined in parts by the optimality of the follower problem (BP-F):


```math
	\begin{align}
	\mathcal{F}:= \{(x,y):G(x,y)\leq 0,x\in X, y\in \Psi(x)\}.

	\end{align}
```
This set is also called the inducible region. Further, we donate the shared constraint set by 

```math
	\begin{align}
	\Omega:= \{(x,y)\in X\times Y: G(x,y)\leq 0, g(x,y)\leq 0\},
	\end{align}
```
and its projection onto the ``x``-space by ``\Omega_x:=\{x: \exists y \text{with} (x,y)\in \Omega\}``. Using this notation, problem (BP) can also be written equivalently as 

```math
	\begin{align}
	&\textbf{(BP-OVF)}\\
	&\min_{x,y} F(x,y),\\
 & s.t.\\ 
& (x,y)\in\Omega,\\
& f(y)\leq \varphi(x).\\
	\end{align}
```
This problem is often referred to as the optimal-value-function reformulation of problem (BP). Most solution approaches for problem (BP) rely on a single-level reformulation similar to problem (BP-OVF). An important relaxation of problem (BP) is given by the high-point relaxation

```math
	\begin{align}
	&\textbf{(HPR)}\\
	&\min_{x,y} F(x,y),\\
 & s.t.\\ 
& (x,y)\in\Omega,\\
	\end{align}
```
which abstracts from the optimality of the follower. Its optimal objective value provides a lower bound on the optimal objective value of problem (BP).

Finally, we introduce a naming convention that we use to denote the classes of bilevel problems that we consider in this research. We therefore introduce the leader problem 

```math
	\begin{align}
	&\textbf{(ILP)}\\
	&\min_{x,y} F(x,y),\\
 & s.t.\\ 
& G(x,y) \leq 0,\\
& x\in X,\\
	\end{align}
```
and the independent follower problem


```math
	\begin{align}
	&\textbf{(IFP)}\\
	&\min_{x,y} f(x,y),\\
 & s.t.\\ 
& g(x,y) \leq 0,\\
& y\in Y.\\
	\end{align}
```
For linear bilevel problems (LBPs), both problems are linear problems (LPs). We thus call LBPs as LP-LP bilevel problems. Similarly, we consider, e.g., MILP-MILP problems, in which both problems (ILP) and (IFP) are mixed- integer linear problems (MILPs). Further, for the class of convex MIQP-QP problems, all involved constraints are assumed to be linear in x and y, and the two objective functions F and f are assumed to be convex-quadratic in x and y. This renders Problem (ILP) a convex mixed-integer quadratic problem (MIQP) and Problem (IFP) a convex quadratic problem (QP).


## Computational Challenges

Bilevel problems possess interesting properties and are computationally challenging due to their hierarchical structure. This is demonstrated by the following example.

**Example 1.** Consider the bilevel problem

```math
\begin{align}
&\min_{x,y} F(x,y)= x+6y,&\\
 & s.t.&\\ 
& -x+5y\leq 12.5,&x\geq 0,\\
& y\in \arg\min_{\bar{y}}\{f(x,\bar{y})=\bar{y}: 2x-\bar{y}\geq 0, -x-\bar{y}\geq -6, -x+6\bar{y}\geq -3, x+3\bar{y}\geq 3 \},\\
\end{align}
```
which is illustrated in Figure (1). The figure reveals several interesting and important obstacles of bilevel programming:

``` @raw html
<figure align = "center">
  <img src="../images/example1.png" alt="Cannot load the image"/ width= "500" >
  <figcaption align = "center"> Example 1: nonconvex feasible set
 </figcaption>
</figure>
```

1. The feasible region of the follower problem corresponds to the gray area. Thus, the follower problem —and therefore the bilevel problem— is infeasible for certain decisions of the leader, e.g., ``x = 0``.
2. The set ``\{(x,y): x \in \Omega_x, y\in \Psi(x) \} `` denotes the optimal follower solutions lifted to the x-y-space, and is given by the green and red facets. This set is nonconvex.
3. The single leader constraint indicated by the dashed line renders certain optimal responses of the follower infeasible. Thus, the bilevel feasible region ``\mathcal{F}`` corresponds to the green facets. Consequently, the feasible set of example (1) is not only nonconvex but also disconnected.
4. The optimal solution of example (1) is (3/7, 6/7) with objective function value 39/7. In contrast, ignoring the follower objective, i.e., solving the high-point relaxation (HPR), yields the optimal solution (3, 0) with objective function value 3. Note that the latter point is not bilevel feasible.


Example (1) indicates that bilevel problems are intrinsically nonconvex due to their hierarchical structure. It has been proven that LP-LP problems are strongly NP-hard; even checking local optimality of LP-LP problems is NP-hard. 


##  Linear and Convex Follower Problems

In this section, we take a closer look at bilevel problems with linear or convex followers. This is the class of bilevel problems that we mainly consider in this research. In case of LP-LP problems, we have the following property.

!!! note "Theorem 1"
    If the set of optimal solutions of an LP-LP problem is nonempty, then it contains at least one vertex of the shared constraint set ``\Omega``.


Thus, LP-LP problems can be solved by vertex enumeration of the shared constraint set ``\Omega``. A more general class contains problems with convex parametric follower problems (BP-F). In this case, the functions ``f(x,.)`` and ``g(x,·)`` are restricted to be convex for fixed decisions ``x`` of the leader and ``Y = \mathbb{R}^m``. If the para- metric follower problem (BP-F) satisfies a constraint qualification, then it can be replaced by its necessary and sufficient KKT conditions. Thus, one can transform Problem (BP) to

```math
\begin{align}
&\textbf{(BP-KKT)}&\\
&\min_{x,y,\lambda} F(x,y),&\\
 & s.t.&\\ 
& G(x,y) \leq 0 & x\in X,\\
& g(x,y) \leq 0,&\\
& \nabla_y \mathcal{L}(x,y,\lambda)=0, &\lambda \geq 0,\\
& \lambda_i g_i(x,\lambda)=0, &i=1,\dots,\ell,
\end{align}
```
in which

```math
\begin{align}
\mathcal{L}(x,y,\lambda)=f(y)+ \lambda^T g(x,y)
\end{align}
```
is the Lagrangian function associated with the follower problem. Problem (BP-KKT) is equivalent to Problem (BP) in the following sense [kleinert2021algorithms](@cite).

!!! note "Theorem 2" 
    Assume that the parametric follower problem (BP-F) is convex and that a constraint qualification holds for each ``x \in \Omega_x``. Then, for every optimal solution ``(x_∗, y_∗)`` of Problem (BP) there exists a point ``λ_∗`` such that ``(x_∗, y_∗, \lambda_∗)`` is an optimal solution of Problem (BP-KKT). Vice versa, for every optimal solution ``(\tilde{x}, \tilde{y}, \tilde{\lambda}) `` of Problem (BP-KKT), the point ``(\tilde{x}, \tilde{y}) `` is an optimal solution of Problem (BP).



## References


```@bibliography
```