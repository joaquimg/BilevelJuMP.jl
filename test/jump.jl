jump_01vec(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config()) = _jump_01(optimizer, true, mode, config)
jump_01(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config()) = _jump_01(optimizer, false, mode, config)
function _jump_01(optimizer, vectorized::Bool, mode, config)

    atol = config.atol

    # config.bound_hint = true

    # min -4x -3y
    # s.t.
    # y = argmin_y y
    #      2x + y <= 4
    #       x +2y <= 4
    #       x     >= 0
    #           y >= 0
    #
    # sol: x = 2, y = 0
    # obj_upper = -8
    # obj_lower =  0

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)
    BilevelJuMP.set_copy_names(model)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @objective(Lower(model), Min, y)

    if vectorized
        @constraints(Lower(model), begin
            c1, 2x+y <= 4
            c2, x+2y <= 4
            c3, x >= 0
            c4, y >= 0
        end)
    else
        @constraint(Lower(model), c1, 2x+y <= 4)
        @constraint(Lower(model), c2, x+2y <= 4)
        @constraint(Lower(model), c3, x >= 0)
        @constraint(Lower(model), c4, y >= 0)
    end

    if config.bound_hint
        for cref in [c1, c2, c3, c4]
            BilevelJuMP.set_dual_upper_bound_hint(cref, 10)
            BilevelJuMP.set_dual_lower_bound_hint(cref, -10)
        end
        BilevelJuMP.set_primal_lower_bound_hint(x, -1)
        BilevelJuMP.set_primal_lower_bound_hint(y, -1)
        BilevelJuMP.set_primal_upper_bound_hint(x, 5)
        BilevelJuMP.set_primal_upper_bound_hint(y, 5)
    end

    # solve twice to check robustness
    optimize!(model)
    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test objective_value(model) ≈ -8 atol=atol

    @test value(x) ≈  2 atol=atol
    @test value(y) ≈  0 atol=atol

    # @test dual(c1) ≈ [0] atol=atol #NLP fail
    @test dual(c2) ≈ [0] atol=atol
    @test dual(c3) ≈ [0] atol=atol
    # @test dual(c4) ≈ [1] atol=atol #NLP fail

    # solve again to check robustness
    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test objective_value(model) ≈ -8 atol=atol
    @test objective_value(Upper(model)) ≈ -8 atol=atol

    # @test JuMP.relative_gap(model) ≈ 0.0 atol=atol
    # @test JuMP.relative_gap(Upper(model)) ≈ 0.0 atol=atol
    # @test JuMP.dual_objective_value(model) ≈ -8 atol=atol
    # @test JuMP.objective_bound(model) ≈ -8 atol=atol

    @test value(x) ≈  2 atol=atol
    @test value(y) ≈  0 atol=atol

    # @test dual(c1) ≈ [0] atol=atol #NLP fail
    @test dual(c2) ≈ [0] atol=atol
    @test dual(c3) ≈ [0] atol=atol
    # @test dual(c4) ≈ [1] atol=atol #NLP fail

    tp = JuMP.objective_function_type(Lower(model))
    JuMP.objective_function(Lower(model), tp)

    # display(x)
    # display(c2)
    # display(model)
    # display(Upper(model))
    # display(Lower(model))

    JuMP.set_objective_coefficient(Upper(model), x, -5)
    JuMP.set_objective_coefficient(Lower(model), x, +1)

    # solve again to check robustness
    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test objective_value(model) ≈ -10 atol=atol
    @test objective_value(Upper(model)) ≈ -10 atol=atol
    @test objective_value(Lower(model)) ≈ 2 atol=atol

    # @test JuMP.relative_gap(model) ≈ 0.0 atol=atol
    # @test JuMP.relative_gap(Upper(model)) ≈ 0.0 atol=atol
    # @test JuMP.dual_objective_value(model) ≈ -8 atol=atol
    # @test JuMP.objective_bound(model) ≈ -8 atol=atol

    @test value(x) ≈  2 atol=atol
    @test value(y) ≈  0 atol=atol

end

function jump_02(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 6)
    @variable(Lower(model), y, start = 2)

    @objective(Upper(model), Min, x + 3y)

    @objective(Lower(model), Min, -y)

    @constraint(Lower(model), c1, x+y <= 8)
    @constraint(Lower(model), c2, x+4y >= 8)
    @constraint(Lower(model), c3, x+2y <= 13)
    @constraint(Lower(model), c4, x >= 1)
    @constraint(Lower(model), c5, x <= 6)
    @constraint(Lower(model), c6, y >= 0)

    if config.bound_hint
        for cref in [c1, c2, c3, c4, c5, c6]
            BilevelJuMP.set_dual_upper_bound_hint(cref, 10)
            BilevelJuMP.set_dual_lower_bound_hint(cref, -10)
        end
        BilevelJuMP.set_primal_upper_bound_hint(y, 10)
        BilevelJuMP.set_primal_lower_bound_hint(y, -1)
        BilevelJuMP.set_primal_lower_bound_hint(x, 0)
        BilevelJuMP.set_primal_upper_bound_hint(x, 7)
    end

    JuMP.set_dual_start_value(c1, -1)
    JuMP.set_dual_start_value(c2, 0)
    JuMP.set_dual_start_value(c3, 0)
    JuMP.set_dual_start_value(c4, 0)
    JuMP.set_dual_start_value(c5, -1)
    JuMP.set_dual_start_value(c6, 0)

    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ALMOST_LOCALLY_SOLVED]

    @test objective_value(model) ≈ 12 atol=atol
    @test BilevelJuMP.lower_objective_value(model) ≈ -2 atol=atol

    @test value(x) ≈ 6 atol=atol
    @test value(y) ≈ 2 atol=atol

    @test dual(c1) ≈ [-1] atol=atol
    @test dual(c2) ≈ [0] atol=atol
    @test dual(c3) ≈ [0] atol=atol
    @test dual(c4) ≈ [0] atol=atol
    # @show dual(c5)# ≈ [0] atol=atol
    @test dual(c6) ≈ [0] atol=atol
    @test value(c1) ≈ 8 atol=atol
    @test value(c2) ≈ 14 atol=atol
    @test value(c3) ≈ 10 atol=atol
    @test value(c4) ≈ 6 atol=atol
    @test value(c5) ≈ 6 atol=atol
    @test value(c6) ≈ 2 atol=atol

end


#=
    From:
    Foundations of Bilevel Programming
    by: Stephan Dempe
    in: Nonconvex Optimization and Its Applications
=#

# obs: example 2 is from the book

# Cap 3.2, Pag 25
jump_03(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config()) = _jump_03(optimizer, false, mode, config)
jump_03_vec(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config()) = _jump_03(optimizer, true, mode, config)
function _jump_03(optimizer, vec::Bool, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Lower(model), x)
    @variable(Upper(model), y)
    if start
        JuMP.set_start_value(x, 3.5*8/15)
        JuMP.set_start_value(y, 8/15)
    end

    @objective(Upper(model), Min, 3x + y)
    @constraint(Upper(model), u1, x <= 5)
    @constraint(Upper(model), u2, y <= 8)
    @constraint(Upper(model), u3, y >= 0)

    @objective(Lower(model), Min, -x)

    if vec
        A = [
             1   1;
            -4  -1;
             2   1;
             2  -7;
        ]
        b = [8, -8, 13, 0]
        @constraint(Lower(model), l,  A*[x, y] .<= b)
        l1 = l[1]
        l3 = l[3]
    else
        @constraint(Lower(model), l1,  x +  y <= 8)
        @constraint(Lower(model), l2, 4x +  y >= 8)
        @constraint(Lower(model), l3, 2x +  y <= 13)
        @constraint(Lower(model), l4, 2x - 7y <= 0)
    end

    if config.bound_hint
        for c in [l1, l2, l3, l4]
            BilevelJuMP.set_dual_upper_bound_hint(c, 15)
            BilevelJuMP.set_dual_lower_bound_hint(c, -15)
        end
        BilevelJuMP.set_primal_lower_bound_hint(x, -10)
        BilevelJuMP.set_primal_upper_bound_hint(x, 6)
        BilevelJuMP.set_primal_lower_bound_hint(y, -1)
        BilevelJuMP.set_primal_upper_bound_hint(y, 9)
    end

    optimize!(model)

    # @test primal_status(model) == MOI.FEASIBLE_POINT
    # @test primal_status(Upper(model)) == MOI.FEASIBLE_POINT
    # @test primal_status(Lower(model)) == MOI.FEASIBLE_POINT

    # @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test objective_value(model) ≈ 3* (3.5*8/15) + (8/15) atol=atol
    @test BilevelJuMP.lower_objective_value(model) ≈ -3.5*8/15 atol=atol
    @test objective_value(Lower(model)) ≈ -3.5*8/15 atol=atol

    @test value(x) ≈ 3.5*8/15 atol=atol
    @test value(y) ≈ 8/15 atol=atol
    @test value(u1) ≈ 3.5*8/15 atol=atol
    @test value(l1) ≈ 4.5*8/15 atol=atol

    # @test JuMP.dual_status(Lower(model)) == MOI.FEASIBLE_POINT

    @test dual(l1) ≈ [0] atol=atol
    # @test dual(l2) #≈ [0] atol=atol
    @test dual(l3) ≈ [0] atol=atol
    # @test dual(l4) #≈ [0] atol=atol

    if typeof(mode) <: BilevelJuMP.ProductMode
        # @test JuMP.dual_status(Upper(model)) == MOI.FEASIBLE_POINT
        @test dual(u1) ≈ 0 atol=atol
    end


end
# change the bound on x to lower level
function jump_04(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Lower(model), x, start = 3.5*8/15)
    @variable(Upper(model), y, start = 8/15)

    @objective(Upper(model), Min, 3x + y)
    @constraint(Upper(model), y <= 8)
    @constraint(Upper(model), y >= 0)
    
    @objective(Lower(model), Min, -x)
    
    @constraint(Lower(model),  x +  y <= 8)
    @constraint(Lower(model), 4x +  y >= 8)
    @constraint(Lower(model), 2x +  y <= 13)
    @constraint(Lower(model), 2x - 7y <= 0)
    @constraint(Lower(model), x <= 5)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ 3* (3.5*8/15) + (8/15) atol=1e-4

    @test value(x) ≈ 3.5*8/15 atol=1e-4
    @test value(y) ≈ 8/15 atol=1e-4

end

# Sec 3.3 , pag 30 -> product of x and y in lower level objective

# Sec 3.4.1 , pag 32
function jump_05(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Lower(model), x[i=1:2])
    @variable(Upper(model), y)

    @objective(Upper(model), Min, 2x[2] - y)
    @constraint(Upper(model), y <= 3)
    @constraint(Upper(model), y >= 0)
    
    @objective(Lower(model), Min, -x[1])
    
    @constraint(Lower(model),  100x[1] -x[2] <= 1)
    @constraint(Lower(model),  x[2] <= y)
    @constraint(Lower(model), x[1] >= 0)
    @constraint(Lower(model), x[2] >= 0)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ 0 atol=1e-5

    @test value(x[1]) ≈ 1/100 atol=atol
    @test value(x[2]) ≈ 0 atol=atol
    @test value(y) ≈ 0 atol=1e-5

end

# sec 3.5.2.2 pag 44 -> product

# sec 3.7 pag 59
function jump_3SAT(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    n = 7 # maximum literals
    clauses = [[1,2,3],[-1,-4,3],[7,-6,4],[5,6,7]]

    @variable(Lower(model), x[i=1:n])
    @variable(Upper(model), ya[i=1:n])
    @variable(Upper(model), yb[i=1:n])
    @variable(Upper(model), z)
    # @variable(Upper(model), z)

    @objective(Upper(model), Min, sum(x[i] for i in 1:n) - z)
    @constraint(Upper(model), ca, z <= 1)
    @constraint(Upper(model), cb, z >= 0)
    @constraint(Upper(model), c1[i=1:n], ya[i] >= 0)
    @constraint(Upper(model), c2[i=1:n], ya[i] <= 1)
    @constraint(Upper(model), c3[i=1:n], yb[i] >= 0)
    @constraint(Upper(model), c4[i=1:n], yb[i] <= 1)
    @constraint(Upper(model), c5[i=1:n], ya[i] + yb[i] == 1)
    @constraint(Upper(model), cc[k in eachindex(clauses)],
        sum(i > 0 ? ya[i] : yb[-i] for i in clauses[k]) >= z)

    @objective(Lower(model), Min, -sum(x[i] for i in 1:n))

    @constraint(Lower(model), b1[i=1:n], x[i] >= 0)
    @constraint(Lower(model), b2[i=1:n], x[i] <= ya[i])
    @constraint(Lower(model), b3[i=1:n], x[i] <= yb[i])

    JuMP.set_start_value.(x, 0)
    JuMP.set_start_value.(ya, 1)
    JuMP.set_start_value.(yb, 0)
    JuMP.set_start_value(z, 1)
    for i in 1:n
        JuMP.set_dual_start_value.(b1, 0)
        JuMP.set_dual_start_value.(b2, 0)
        JuMP.set_dual_start_value.(b3, -1)
    end

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ -1 atol=atol

    # 3SAT is yese IFF obj = -1

    for i in 1:n
        @test value(x[i]) ≈ min(value(ya[i]), value(yb[i])) atol=atol
        @test -atol <= value(x[i]) <= +atol
        @test 1 - atol <= value(ya[i]) <= 1 + atol || -atol <= value(ya[i]) <= +atol
        @test 1 - atol <= value(yb[i]) <= 1 + atol || -atol <= value(yb[i]) <= +atol
    end
    @test value(z) ≈ 1 atol=atol

end

# sec 5.1 pag 121 -> product

# sec 5.1 pag 127
function jump_quad_01_a(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Lower(model), x, start = 0)
    @variable(Upper(model), y, start = 0)

    @objective(Upper(model), Min, x^2 + y)
    @constraint(Upper(model), u1, -x -y <= 0)

    @objective(Lower(model), Min, x)
    @constraint(Lower(model), l1, x >= 0)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ 0 atol=atol
    @test value(x) ≈ 0 atol=atol
    @test value(y) ≈ 0 atol=atol

end
function jump_quad_01_b(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Lower(model), x, start = 0.5)
    @variable(Upper(model), y, start = -0.5)

    @objective(Upper(model), Min, x^2 + y)
    
    @objective(Lower(model), Min, x)
    @constraint(Lower(model), l1, -x -y <= 0)
    @constraint(Lower(model), l2, x >= 0)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ 0.5^2 - 0.5 atol=atol
    @test value(x) ≈ 0.5 atol=atol
    @test value(y) ≈ -0.5 atol=atol

end
function jump_quad_01_c(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Lower(model), x)
    @variable(Upper(model), y)

    @objective(Upper(model), Min, x^2 + y)
    @constraint(Upper(model), -x -y <= 0)
    @constraint(Upper(model), x >= 0)
    
    @objective(Lower(model), Min, x)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    # lower level migh not be always feasible
    # KKT and global methods should work

end

# sec 5.1 pag 126 -> product

# sec 5.2 pag 130 -> quadracti second level! (good test for new KKT)
# sec 5.5 pag 149 -> quadracti 
# sec 5.7.1 pag 159, 161, 169, 171 -> quadracti 

# sec 6.1 pag 196 -> quadratic objective in second level
# sec 7.2 pag 226,227, 233 -> quad


# sec 8.1 pag 255 example from [279]
# only the second level is described
function jump_int_01(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Lower(model), x)
    @variable(Upper(model), y)

    @objective(Upper(model), Min, 0)
    # @constraint(Upper(model), -x -y <= 0)
    # @constraint(Upper(model), x >= 0)

    # include integrality

    @objective(Lower(model), Min, x)

    @constraint(Lower(model), x + y <= 2)
    @constraint(Lower(model), x - y <= 2)
    @constraint(Lower(model), -4x + 5y <= 10)
    @constraint(Lower(model), -4x - 5y <= 10)

    optimize!(model)

    primal_status(model)

    termination_status(model)


end

# sec 8.1 pag 257
function jump_int_02(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Lower(model), x)
    @variable(Upper(model), y)

    @objective(Upper(model), Min, -10x - y)

    # include integrality on Y

    @objective(Lower(model), Min, x)

    @constraint(Lower(model), 20x - 25y <= 30)
    @constraint(Lower(model), 2x + y <= 10)
    @constraint(Lower(model), -x + 2y <= 15)
    @constraint(Lower(model), 10x + 2y >= 15)

    # DONT include integrality on X (KKT wont work)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    # continuous solution x = 1, y = 8
    # same for integer on upper
    # integer on both levels is 2,2

end

# sec 8.3 pag 265 -> products + integrality on lower

#=
    From:
    Practical Bilevel Optimization
    by: J. F. Bard
    in:
=#


# pag 197 ex 5.1.1
jump_06(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config()) = _jump_06(optimizer, false, mode, config)
jump_06_sv(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config()) = _jump_06(optimizer, true, mode, config)
function _jump_06(optimizer, sv::Bool, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    if sv
        @variable(Upper(model), x >= 0, start = 4)
        @variable(Lower(model), y >= 0, start = 4)
    else
        @variable(Upper(model), x, start = 4)
        @variable(Lower(model), y, start = 4)
    end

    @objective(Upper(model), Min, x - 4y)
    if !sv
        @constraint(Upper(model), x >= 0)
    end

    @objective(Lower(model), Min, y)

    @constraint(Lower(model), -x  - y <= -3)
    @constraint(Lower(model), -2x + y <= 0)
    @constraint(Lower(model), 2x  + y <= 12)
    @constraint(Lower(model), 3x + -2y <= 4) # signs are wrong in some versions of the book
    if !sv
        @constraint(Lower(model), y >= 0)
    # else
    #     @constraint(Lower(model), y == 4)
    #     @constraint(Lower(model), x == 4)
    end

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ -12 atol=atol

    @test value(x) ≈ 4 atol=atol
    @test value(y) ≈ 4 atol=atol

end

# pag 208 ex 5.3.1
function jump_07(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x[i=1:2], start = [0, 0.9][i])
    @variable(Lower(model), y[i=1:3], start = [0, 0.6, 0.4][i])

    @objective(Upper(model), Min,
        -8x[1] -4x[2] + 4y[1] - 40y[2] - 4y[3])
    @constraint(Upper(model), [i=1:2], x[i] >= 0)
    
    @objective(Lower(model), Min,
        x[1]+ 2x[2] + y[1] + y[2] + 2y[3])
    @constraint(Lower(model), [i=1:3], y[i] >= 0)

    @constraint(Lower(model), -y[1] + y[2] + y[3] <= 1)
    @constraint(Lower(model), 2x[1] - y[1] + 2y[2] - 0.5y[3] <= 1)
    @constraint(Lower(model), 2x[2] +2y[1] - y[2] - 0.5y[3] <= 1)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value.(x) ≈ [0, 0.9] atol=atol
    @test value.(y) ≈ [0, 0.6, 0.4] atol=atol

end

# pag 208 ex 5.3.1
function jump_08(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 8/9)
    @variable(Lower(model), y, start = 20/9)

    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), x >= 0)
    
    @objective(Lower(model), Min, -5x - y)
    @constraint(Lower(model), y >= 0)

    @constraint(Lower(model), -x - y/2 <= -2)
    @constraint(Lower(model), -x/4 + y <= 2)
    @constraint(Lower(model), x + y/2 <= 8)
    @constraint(Lower(model), x - 2y <= 4)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 8/9 atol=atol
    @test value(y) ≈ 20/9 atol=atol

end

# pag 271 ex 7.1.1 -> quadratic terms
# pag 281 ex 7.2.2 -> quadratic terms
# pag 302 ex 8.1.1 -> quadratic terms

# pag 304 ex 8.1.2
function jump_09a(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    # degenerate second level
    # its case that show that the KKT approach is OPTIMISTIC

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 0.0)
    @variable(Lower(model), y[i=1:2], start = [0, 1][i])

    @objective(Upper(model), Min, -10.1x + 10y[1] - y[2])
    @constraint(Upper(model), x >= 0)

    @constraint(Upper(model), y[1] >= 0)
    @constraint(Upper(model), y[2] >= 0)
    
    @objective(Lower(model), Min, -y[1] - y[2])
    @constraint(Lower(model), y[1] >= 0)
    @constraint(Lower(model), y[2] >= 0)

    @constraint(Lower(model), x - y[1] <= 1)
    @constraint(Lower(model), x + y[2] <= 1)
    @constraint(Lower(model), y[1] + y[2] <= 1)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 0 atol=atol
    @test value.(y) ≈ [0, 1] atol=atol

end
function jump_09b(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y[i=1:2])

    @objective(Upper(model), Min, -x + 10y[1] - 2y[2])
    @constraint(Upper(model), x >= 0)
    
    @objective(Lower(model), Min, -y[1] - y[2])
    @constraint(Lower(model), y[1] >= 0)
    @constraint(Lower(model), y[2] >= 0)

    @constraint(Lower(model), x - y[1] <= 1)
    @constraint(Lower(model), x + y[2] <= 1)
    @constraint(Lower(model), y[1] + y[2] <= 1)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 0 atol=atol
    @test value.(y) ≈ [0, 1] atol=atol

end

# pag 306 ex 8.1.3 -> quardratics
# pag 336 ex 8.5.1 -> quardratics
# pag 358 ex 8.7.1 -> quardratics


#=
    From:
    Bilevel programming problems
    by: S. Dempe et al.
    in: Energy systems
=#

# pag 4 ex 1.2 -> product

# pag 9 ex 1.4 TODO obtain solution
function jump_10(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    n = 10
    f = 100
    b_l = 0
    b_u = 50

    c_lw = 
    [10 30 8 60 4 16 32 30 120 6]
    a_lw =
    [5 3 2 5 1 8 4 3 6 3]
    c_up =
    [10 15 -24 20 -40 80 -32 -60 -12 -60]

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), b)
    @variable(Lower(model), x[i=1:n])

    @objective(Upper(model), Min, sum(c_up[i]*x[i] for i in 1:n) + f*b)
    @constraint(Upper(model), b >= b_l)
    @constraint(Upper(model), b <= b_u)

    @objective(Lower(model), Min, sum(c_lw[i]*x[i] for i in 1:n) )
    @constraint(Lower(model), [i=1:n], x[i] >= 0)
    @constraint(Lower(model), [i=1:n], x[i] <= 1)
    @constraint(Lower(model), sum(a_lw[i]*x[i] for i in 1:n) >= b)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    # @test value(x) ≈ 0
    # @test value(y) ≈ [1, 0]

end

# pag 21 ex 2.1
function jump_11a(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 8)
    @variable(Lower(model), y, start = 6)

    @objective(Upper(model), Min, -x - 2y)
    @constraint(Upper(model), 2x - 3y >= -12)
    @constraint(Upper(model), x + y <= 14)
    
    @objective(Lower(model), Min, -y)
    @constraint(Lower(model), -3x + y <= -3)
    @constraint(Lower(model), 3x + y <= 30)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 8 atol=atol
    @test value(y) ≈ 6 atol=atol

end
function jump_11b(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 6)
    @variable(Lower(model), y, start = 8)

    @objective(Upper(model), Min, -x - 2y)
    
    @objective(Lower(model), Min, -y)
    @constraint(Lower(model), 2x - 3y >= -12)
    @constraint(Lower(model), x + y <= 14)
    @constraint(Lower(model), -3x + y <= -3)
    @constraint(Lower(model), 3x + y <= 30)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 6 atol=atol
    @test value(y) ≈ 8 atol=atol

end

# pag 45 ex 3.3
function jump_12(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    for a in [0 0.1 0.2]
        atol = config.atol
    
        MOI.empty!(optimizer)
        model = BilevelModel(()->optimizer, mode = mode)

        @variable(Upper(model), x)
        @variable(Lower(model), y)
        @variable(Lower(model), z)

        @objective(Upper(model), Min, 0.5x - y + 3z)
        
        @objective(Lower(model), Min, -y - z)
        @constraint(Lower(model), x + y <= 1 + a)
        @constraint(Lower(model), -x + y <= 1)
        @constraint(Lower(model), z <= 1)
        @constraint(Lower(model), z >= 0)

        MOI.empty!(optimizer)
        @test MOI.is_empty(optimizer)

        optimize!(model)

        primal_status(model)

        termination_status(model)

        @test value(x) ≈ a/2 atol=atol
        @test value(y) ≈ 1 + a/2 atol=atol
        @test value(z) ≈ 1 atol=atol
    end
end

# pag 45 ex 3.3
function jump_13_quad(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 3/2)
    @variable(Lower(model), y, start = 1/2)

    @objective(Upper(model), Min, (x -1 )^2 + y^2)
    @constraint(Upper(model), x >= -2)
    @constraint(Upper(model), x <= +2)

    @objective(Lower(model), Min, -y)
    @constraint(Lower(model), x + y <= 2)
    @constraint(Lower(model), -x + y <= 2)
    @constraint(Lower(model), y >= 0)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 3/2 atol=atol
    @test value(y) ≈ 1/2 atol=atol
end

# pag 290 ex 8.2
function jump_14(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Upper(model), y)
    @variable(Lower(model), z)

    @objective(Upper(model), Min, x - 2y + z)
    @constraint(Upper(model), x + y +z >= 15)
    @constraint(Upper(model), x <= 10)
    @constraint(Upper(model), y <= 10)
    @constraint(Upper(model), z <= 10)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), y >= 0)
    @constraint(Upper(model), z >= 0)

    @objective(Lower(model), Min, 2x-y+z)
    @constraint(Lower(model), x + y -z <= 5)
    @constraint(Lower(model), z >= 0)
    @constraint(Lower(model), z <= 10)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 0 atol=atol
    @test value(y) ≈ 10 atol=atol
    @test value(z) ≈ 5 atol=atol
end

# pag 300 ex 8.5.2
function jump_15a_INT(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)
    @variable(Lower(model), z)

    @objective(Upper(model), Min, x - 2y + z)
    @constraint(Upper(model), x <= 1)
    @constraint(Upper(model), x >= 0)
    # ADD INTEGRALITY (binary) ON X
    @constraint(Upper(model), y <= 100)
    @constraint(Upper(model), z >= 0)
    @constraint(Upper(model), z <= 100)
    @constraint(Upper(model), y >= 0)

    @objective(Lower(model), Min, - 60y - 8z)
    @constraint(Lower(model), 10x + 2y + 3z <= 225)
    @constraint(Lower(model),  5x + 3y + 0z <= 230)
    @constraint(Lower(model),  5x + 0y + 1z <= 85)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 1 atol=atol
    @test value(y) ≈ 75 atol=atol
    @test value(Z) ≈ 21+2/3 atol=atol
end
function jump_15b_INT(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Upper(model), y)
    @variable(Lower(model), z)

    @objective(Upper(model), Min, x - 2y + z)
    @constraint(Upper(model), x <= 1)
    @constraint(Upper(model), x >= 0)
    # ADD INTEGRALITY (binary) ON X
    @constraint(Upper(model), y <= 100)
    @constraint(Upper(model), z >= 0)
    @constraint(Upper(model), z <= 100)
    @constraint(Upper(model), y >= 0)

    @objective(Lower(model), Min, - 60y - 8z)
    @constraint(Lower(model), 10x + 2y + 3z <= 225)
    @constraint(Lower(model),  5x + 3y + 0z <= 230)
    @constraint(Lower(model),  5x + 0y + 1z <= 85)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 1 atol=atol
    @test value(y) ≈ 75 atol=atol
    @test value(Z) ≈ 21+2/3 atol=atol
end

#=
    Princeton Handbook of Test Problems in Local and Global Optimization
    Floudas c., Pardalos P., et al.
    HTP
=#

# 9.2.2
function jump_HTP_lin01(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y[i=1:2])
    if start
        JuMP.set_start_value(x, 5.0)
        JuMP.set_start_value(y[1], 4.0)
        JuMP.set_start_value(y[2], 2.0)
    end

    @objective(Upper(model), Min, -x - 3y[1] + 2y[2])
    @constraint(Upper(model), x <= 8)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), [i=1:2], y[i] <= 4)
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    @objective(Lower(model), Min, - y[1])
    @constraint(Lower(model), -y[1] <= 0)
    @constraint(Lower(model),  y[1] <= 4)
    @constraint(Lower(model), -2x + 2y[1] + 4y[2] <= 16)
    @constraint(Lower(model),  8x + 3y[1] - 2y[2] <= 48)
    @constraint(Lower(model), -2x +  y[1] - 3y[2] <= -12)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 5 atol=atol
    @test value.(y) ≈ [4, 2] atol=atol
end

# 9.2.3
function jump_HTP_lin02(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 4)
    @variable(Lower(model), y, start = 4)

    @objective(Upper(model), Min, -x - 3y)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), y >= 0)

    @objective(Lower(model), Min, y)
    @constraint(Lower(model), c1, -y <= 0)
    @constraint(Lower(model), c2, -x + y <= 3)
    @constraint(Lower(model),   x + 2y <= 12)
    @constraint(Lower(model),  4x -  y <= 12)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 4 atol=atol
    @test value(y) ≈ 4 atol=atol
    @test dual(c1) ≈ [0] atol=atol
    @test dual(c2) ≈ [0] atol=atol
end

# 9.2.4 - parg 211
jump_HTP_lin03(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config()) = _jump_HTP_lin03(optimizer, false, mode, config)
jump_HTP_lin03_vec(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config()) = _jump_HTP_lin03(optimizer, true, mode, config)
function _jump_HTP_lin03(optimizer, vec::Bool, mode, config)
    # send y to upper level

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x[i=1:2], start = [0, 0.9][i])
    @variable(Lower(model), y[i=1:6], start = [0, 0.6, 0.4, 0, 0, 0][i])

    @objective(Upper(model), Min,
        4y[1] - 40y[2] -4y[3] -8x[1] -4x[2])
    @constraint(Upper(model), [i=1:2], x[i] >= 0)
    @constraint(Upper(model), [i=1:6], y[i] >= 0)

    @objective(Lower(model), Min,
        y[1] + y[2] + 2y[3] +x[1] +2x[2])

    @constraint(Lower(model), [i=1:6], y[i] >= 0)
    if vec
        H1 = [ -1  1  1   1  0  0;
               -1  2 -1/2 0  1  0;
                2 -1 -1/2 0  0  1
            ]
        H2 = [0 0;
              2 0;
              0 2
            ]
        b = [1, 1, 1]
        @constraint(Lower(model), H1*y + H2*x .== b)
    else
        @constraint(Lower(model), c1, -y[1] +  y[2] +       y[3] + y[4]         == 1)
        @constraint(Lower(model), c2, -y[1] + 2y[2] - (1/2)*y[3] + y[5] + 2x[1] == 1)
        @constraint(Lower(model), c3, 2y[1] -  y[2] - (1/2)*y[3] + y[6] + 2x[2] == 1)
    end

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value.(x) ≈ [0, 0.9] atol=atol
    @test value.(y) ≈ [0, 0.6, 0.4, 0, 0, 0] atol=atol
end

# 9.2.5
function jump_HTP_lin04(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, x - 4y)
    # @constraint(Upper(model), x >= 0)
    # @constraint(Upper(model), y >= 0)

    @objective(Lower(model), Min, y)
    @constraint(Lower(model), -2x +  y <= 0)
    @constraint(Lower(model),  2x + 5y <= 108)
    @constraint(Lower(model),  2x - 3y <= -4)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 19 atol=atol
    @test value(y) ≈ 14 atol=atol
end

# 9.2.6
function jump_HTP_lin05(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 0)
    @variable(Lower(model), y[i=1:2], start = [0, 1][i])

    @objective(Upper(model), Min, -x + 10y[1] - y[2])
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    @objective(Lower(model), Min, -y[1] - y[2])
    @constraint(Lower(model),    x -  y[1] <= 1)
    @constraint(Lower(model),    x +  y[2] <= 1)
    @constraint(Lower(model), y[1] +  y[2] <= 1)
    @constraint(Lower(model),  -y[1] <= 0)
    @constraint(Lower(model),  -y[2] <= 0)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    # book solution looks incorrect
    # the actual solutions seems to be
    @test value(x) ≈ 0 atol=atol
    @test value.(y) ≈ [0, 1] atol=atol
end

# 9.2.7
function jump_HTP_lin06(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 16)
    @variable(Lower(model), y, start = 11)

    @objective(Upper(model), Min, -x - 3y)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), y >= 0)

    @objective(Lower(model), Min, -x + 3y)
    @constraint(Lower(model),   - x - 2y <= -10)
    @constraint(Lower(model),     x - 2y <= 6) # sign of 2y was wrong on book
    @constraint(Lower(model),    2x -  y <= 21)
    @constraint(Lower(model),     x + 2y <= 38)
    @constraint(Lower(model),    -x + 2y <= 18)
    @constraint(Lower(model),         -y <= 0)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    # book solution looks incorrect
    # the actual solutions seems to be
    @test value(x) ≈ 16 atol=atol
    @test value(y) ≈ 11 atol=atol
end

# 9.2.8 - parg 216
function jump_HTP_lin07(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol
    rtol = config.rtol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x[i=1:2], start = [0, 0.9][i])
    @variable(Lower(model), y[i=1:3], start = [0, 0.6, 0.4][i])

    @objective(Upper(model), Min,
        -8x[1] -4x[2] + 4y[1] - 40y[2] + 4y[3])
        # the sign of 4y[3] seems off, but solution is the same
        # model from the book left as is
    @constraint(Upper(model), [i=1:2], x[i] >= 0)
    @constraint(Upper(model), [i=1:3], y[i] >= 0)

    @objective(Lower(model), Min,
        x[1] + 2x[2] + y[1] + y[2] + 2y[3])

    @constraint(Lower(model), [i=1:3], y[i] >= 0)

    @constraint(Lower(model),         -  y[1] +  y[2] +       y[3] <= 1)
    @constraint(Lower(model), + 2x[1] -  y[1] + 2y[2] - (1/2)*y[3] <= 1)
    @constraint(Lower(model), + 2x[2] + 2y[1] -  y[2] - (1/2)*y[3] <= 1)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value.(x) ≈ [0, 0.9] atol=atol rtol=rtol
    @test value.(y) ≈ [0, 0.6, 0.4] atol=atol rtol=rtol
end

# 9.2.9 - parg 217
function jump_HTP_lin08(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x[i=1:2], start = [2, 0][i])
    @variable(Lower(model), y[i=1:2], start = [1.5, 0][i])

    @objective(Upper(model), Min,
        -2x[1] + x[2] + 0.5*y[1])
    @constraint(Upper(model), [i=1:2], x[i] >= 0)
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    @objective(Lower(model), Min,
        x[1] + x[2] - 4y[1] + y[2])

    @constraint(Lower(model), [i=1:2], y[i] >= 0)

    @constraint(Lower(model), - 2x[1]         +  y[1] -  y[2] <= -2.5)
    @constraint(Lower(model), +  x[1] - 3x[2]         +  y[2] <= 2)
    @constraint(Lower(model), +  x[1] +  x[2]                 <= 2)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    # solution frm the book is incorrect
    # using solution from: https://www.gams.com/latest/emplib_ml/libhtml/emplib_flds918.html
    @test value.(x) ≈ [2, 0] atol=atol
    @test value.(y) ≈ [1.5, 0] atol=atol
end

# 9.2.10
function jump_HTP_lin09(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), y >= 0)

    @objective(Lower(model), Min, -5x - y)
    @constraint(Lower(model),   - x - 0.5*y <= -2)
    @constraint(Lower(model), -0.25*x + y <= 2) # sign of 2y was wrong on book
    @constraint(Lower(model),     x + 0.5*y <= 8)
    @constraint(Lower(model),     x - 2y <= 2)
    @constraint(Lower(model),         -y <= 0)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 8/9 atol=atol#0.888_888_888_888
    @test value(y) ≈ 20/9 atol=atol# 2.222_222_222_222
end

# 9.2.11 - parg 219
function jump_HTP_lin10(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x[i=1:2], start = [2, 0][i])
    @variable(Lower(model), y[i=1:2], start = [1.5, 0][i])

    @objective(Upper(model), Min,
        -2x[1] + x[2] + 0.5*y[1])
    @constraint(Upper(model), [i=1:2], x[i] >= 0)
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    # this upper constraint is missing in the book but is correct in the GAMS file
    @constraint(Upper(model), +  x[1] +  x[2]  <= 2)

    @objective(Lower(model), Min,
        x[1] + x[2] - 4y[1] + y[2])

    @constraint(Lower(model), [i=1:2], y[i] >= 0)

    @constraint(Lower(model), - 2x[1]         +  y[1] -  y[2] <= -2.5)
    @constraint(Lower(model), +  x[1] - 3x[2]         +  y[2] <= 2)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    # solution frm the book is incorrect
    # using solution from: https://www.gams.com/latest/emplib_ml/libhtml/emplib_flds918.html
    @test value.(x) ≈ [2, 0] atol=atol
    @test value.(y) ≈ [1.5, 0] atol=atol
end

# TODO - add quadratic problems

# 9.3.2 - parg 221
function jump_HTP_quad01(optimizer, is_min, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min,
        (x-5)^2 + (2y+1)^2)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), y >= 0) # only in lowrrin GAMS

    if is_min
        @objective(Lower(model), Min,
            (y-1)^2 -1.5*x*y)
    else
        @objective(Lower(model), Max,
            -((y-1)^2 -1.5*x*y))
    end

    @constraint(Lower(model), -3x +    y <= -3)
    @constraint(Lower(model),   x - 0.5y <= 4)
    @constraint(Lower(model),   x +    y <= 7)
    # @constraint(Lower(model), y >= 0) # GAMS file has this constraint


    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 1 atol=atol
    @test value(y) ≈ 0 atol=atol
end

# 9.3.3- parg 222
function jump_HTP_quad02(optimizer, is_min, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min,
        x^2 + (y-10)^2)
    @constraint(Upper(model), - x + y <= 0)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), x <= 15)
    @constraint(Upper(model), y >= 0)
    @constraint(Upper(model), y <= 20)

    if is_min
        @objective(Lower(model), Min,
            (x + 2y - 30)^2)
    else
        @objective(Lower(model), Max,
            -((x + 2y - 30)^2))
    end

    @constraint(Lower(model),   x +    y <= 20)
    @constraint(Lower(model),     -    y <= 0)
    @constraint(Lower(model),          y <= 20)


    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 10 atol=1e-4
    @test value(y) ≈ 10 atol=1e-4
end

# 9.3.4- parg 223
function jump_HTP_quad03(optimizer, is_min, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x[i=1:2], start = 0)
    @variable(Lower(model), y[i=1:2], start = -10)

    @objective(Upper(model), Min,
        2x[1] + 2x[2] -3y[1] - 3y[2] -60)
    @constraint(Upper(model), x[1] + x[2] + y[1] -2y[2] -40 <= 0)
    @constraint(Upper(model), [i=1:2], x[i] >= 0)
    @constraint(Upper(model), [i=1:2], x[i] <= 50)
    @constraint(Upper(model), [i=1:2], y[i] >= -10)
    @constraint(Upper(model), [i=1:2], y[i] <= 20)

    if is_min
        @objective(Lower(model), Min,
            (-x[1] + y[1] + 40)^2 + (-x[2] + y[2] + 20)^2)
    else
        @objective(Lower(model), Max,
            -((-x[1] + y[1] + 40)^2 + (-x[2] + y[2] + 20)^2))
    end
    # the boo does not contain the 40, it is a 20 there
    # however the solution does not match
    # the file https://www.gams.com/latest/emplib_ml/libhtml/emplib_flds923.html
    # has a 40

    @constraint(Lower(model),  [i=1:2],- x[i] + 2y[i] <= -10)
    @constraint(Lower(model), [i=1:2], y[i] >= -10)
    @constraint(Lower(model), [i=1:2], y[i] <= 20)


    optimize!(model)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ 0 atol=atol

    sol = vcat(value.(x), value.(y))
    @test sol ≈ [0 ; 0 ; -10; -10] || sol ≈ [0 ; 30; -10; 10] #atol=atol
    # gurobi found the second solution  which is actually feasible and
    # has the same objective value
end

# 9.3.5- parg 225
function jump_HTP_quad04(optimizer, is_min, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 3)
    @variable(Lower(model), y[i=1:2], start = [1, 2][i])

    @objective(Upper(model), Min,
        0.5*((y[1]-2)^2+(y[2]-2)^2) )
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    if is_min
        @objective(Lower(model), Min,
            0.5*(y[1]^2) + y[2])
    else
        @objective(Lower(model), Max,
            -(0.5*(y[1]^2) + y[2]))
    end

    @constraint(Lower(model), y[1] + y[2] == x)
    @constraint(Lower(model), [i=1:2], y[i] >= 0)

    optimize!(model)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ 0.5 atol=atol

    @test value(x) ≈ 3 atol=atol
    @test value.(y) ≈ [1 ; 2] atol=atol
end

# 9.3.6- parg 226
function jump_HTP_quad05(optimizer, is_min, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 1)
    @variable(Lower(model), y, start = 3)

    @objective(Upper(model), Min,
        (x-3)^2+(y-2)^2 )
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), x <= 8)

    if is_min
        @objective(Lower(model), Min,
            (y-5)^2 )
    else
        @objective(Lower(model), Max,
            -((y-5)^2) )
    end

    @constraint(Lower(model), -2x+y <= 1)
    @constraint(Lower(model),  x-2y <= 2)
    @constraint(Lower(model),  x+2y <= 14)
    @constraint(Lower(model), y >= 0)


    optimize!(model)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ 5 atol=1e-5

    @test value(x) ≈ 1 atol=1e-5
    @test value(y) ≈ 3 atol=1e-5
end

# 9.3.7- parg 227
function jump_HTP_quad06(optimizer, is_min, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x[i=1:2], start = 0.5)
    @variable(Lower(model), y[i=1:2], start = 0.5)

    @objective(Upper(model), Min,
        x[1]^2 -2x[1] +x[2]^2 -2x[2] +y[1]^2 +y[2]^2)
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    if is_min
        @objective(Lower(model), Min,
            (-x[1] + y[1])^2 + (-x[2] + y[2])^2)
    else
        @objective(Lower(model), Max,
            -((-x[1] + y[1])^2 + (-x[2] + y[2])^2))
    end

    @constraint(Lower(model), [i=1:2], y[i] >= 0.5)
    @constraint(Lower(model), [i=1:2], y[i] <= 1.5)


    optimize!(model)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ -1 atol=atol

    sol = vcat(value.(x), value.(y))
    @test sol ≈ [0.5 ; 0.5; 0.5; 0.5] atol=atol

end
function jump_HTP_quad06b(optimizer, is_min, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # TODO reviews the behaviour comment
    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x[i=1:2], start = 0.0)
    @variable(Lower(model), y[i=1:2], start = 0.5)

    @objective(Upper(model), Min,
        x[1]^2 +x[2]^2 +y[1]^2 - 3y[1] +y[2]^2 - 3y[2])
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    if is_min
        @objective(Lower(model), Min,
            (-x[1] + y[1])^2 + (-x[2] + y[2])^2)
    else
        @objective(Lower(model), Max,
            -((-x[1] + y[1])^2 + (-x[2] + y[2])^2))
    end

    @constraint(Lower(model), [i=1:2], y[i] >= 0.5)
    @constraint(Lower(model), [i=1:2], y[i] <= 1.5)

    optimize!(model)

    #=
    book claims:
    x = y = [0.5; 0.5]*sqrt(3)
    with objective: -2.1961524227066325
    However,
    Xpress reports:
    x = [0.0, 0.0]
    y = [0.5, 0.5]
    with objective: -2.5
    And
    if the upper variable x = [0.0, 0.0]
    then y = [0.5, 0.5]
    sol the solution by xpress is feasible
    xpress is smaller in a min problem
    xpress solution is, at least, better than book
    =#

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ -2.5 atol=atol

    sol = vcat(value.(x), value.(y))
    @test sol ≈ [0.0 ; 0.0; 0.5; 0.5] atol=atol

end


# 9.3.8- parg 228
function jump_HTP_quad07(optimizer, is_min, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min,
        (x-5)^2 + (2y+1)^2)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), y >= 0) # only in lowrrin GAMS

    if is_min
        @objective(Lower(model), Min,
            (y-1)^2 -1.5*x*y)
    else
        @objective(Lower(model), Max,
            -((y-1)^2 -1.5*x*y))
    end

    @constraint(Lower(model), -3x +    y <= -3)
    @constraint(Lower(model),   x - 0.5y <= 4)
    @constraint(Lower(model),   x +    y <= 7)
    @constraint(Lower(model), y >= 0) # only difference from problem 1


    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 1 atol=atol
    @test value(y) ≈ 0 atol=atol
end

# 9.3.9 - parg 229
function jump_HTP_quad08(optimizer, is_min, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # Q objective is not PSD

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 0.25)
    @variable(Lower(model), y, start = 0)

    @objective(Upper(model), Min,
        -(4x-3)*y+(2x+1) )
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), x <= 1)
    @constraint(Upper(model), y >= 0)
    @constraint(Upper(model), y <= 1)

    if is_min
        @objective(Lower(model), Min,
            -(1-4x)*y -(2x+2) )
    else
        @objective(Lower(model), Max,
            -(-(1-4x)*y -(2x+2) ))
    end

    @constraint(Lower(model), y >= 0)
    @constraint(Lower(model), y <= 1)


    optimize!(model)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ 1.5 atol=atol

    @test value(x) ≈ 0.25 atol=atol
    @test value(y) ≈ 0 atol=atol
end

# 9.3.10- parg 230
function jump_HTP_quad09(optimizer, is_min, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 2)
    @variable(Lower(model), y[i=1:2], start = [6, 0][i])

    @objective(Upper(model), Min,
        x+y[2] )
    @constraint(Upper(model), [i=1:2], y[i] >= 0)
    @constraint(Upper(model), x >= 0)

    if is_min
        @objective(Lower(model), Min,
            2y[1]+x*y[2])
    else
        @objective(Lower(model), Max,
            -(2y[1]+x*y[2]))
    end

    @constraint(Lower(model), x + 4 <= y[1] + y[2])
    # this 4 is missing from the book
    # see https://www.gams.com/latest/emplib_ml/libhtml/emplib_flds929.html
    @constraint(Lower(model), [i=1:2], y[i] >= 0)


    optimize!(model)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ 2 atol=atol

    @test value(x) ≈ 2 atol=atol
    @test value.(y) ≈ [6 ; 0] atol=atol
end

#=
    GAMS educational examples by Michael Ferris
=#

# from https://www.gams.com/latest/emplib_ml/libhtml/emplib_jointc1.html
function jump_jointc1(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    ###
    ### PART 1
    ###

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, x)
    @constraint(Upper(model), x >= 1)
    @constraint(Upper(model), y >= +x)
    @constraint(Upper(model), y >= -x)

    # this lower problem leads to x == -x (infeasible because x >= 1)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), y >= -x)

    optimize!(model)

    @test termination_status(model) in [MOI.INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED, MOI.LOCALLY_INFEASIBLE]

    ###
    ### PART 2
    ###

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 1)
    @variable(Lower(model), y, start = 1)

    @objective(Upper(model), Min, x)
    @constraint(Upper(model), x >= 1)
    @constraint(Upper(model), y >= +x)
    @constraint(Upper(model), y >= -x)

    # this lower problem leads to x == -x (infeasible because x >= 1)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), y >= -x)
    @constraint(Lower(model), y >= +x)

    optimize!(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ALMOST_LOCALLY_SOLVED]

    @test value(x) ≈ 1 atol=atol
    @test value(y) ≈ 1 atol=atol
end

# from https://www.gams.com/latest/emplib_ml/libhtml/emplib_jointc2.html
function jump_jointc2(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    ###
    ### PART 1
    ###

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 1)
    @variable(Lower(model), y, start = -1)

    @objective(Upper(model), Max, x)

    @constraint(Upper(model), x >= 1)
    @constraint(Upper(model), x <= 2)
    @constraint(Upper(model), y <= 100)

    @constraint(Upper(model), y >= x - 2)
    @constraint(Upper(model), y >= -x)

    # this lower problem leads to x == -x (infeasible because x >= 1)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), y >= -x)

    optimize!(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test value(x) ≈ +1 atol=atol
    @test value(y) ≈ -1 atol=atol

    ###
    ### PART 2
    ###

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 2)
    @variable(Lower(model), y, start = 0)

    @objective(Upper(model), Max, x)

    @constraint(Upper(model), x >= 1)
    @constraint(Upper(model), x <= 2)
    @constraint(Upper(model), y <= 100)

    @constraint(Upper(model), y >= x - 2)
    @constraint(Upper(model), y >= -x)

    # this lower problem leads to x == -x (infeasible because x >= 1)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), y >= -x)
    @constraint(Lower(model), y >= x - 2)

    optimize!(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test value(x) ≈ 2 atol=atol
    @test value(y) ≈ 0 atol=atol
end

#=
    An Efficient Point Algorithm for Two-Stage Optimization Problem
    J.F. Bard
    Operations Research, 1983
=#
function jump_EffPointAlgo(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())
    # send y to upper level

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x[i=1:6])
    @variable(Lower(model), y[i=1:3])

    @objective(Upper(model), Max,
        2x[1] - x[2] - x[3] + 2x[4] + x[5] -3.5x[6]
        - y[1] -1.5y[2] + 3y[3])
    @constraint(Upper(model), [i=1:6], x[i] >= 0)
    
    @objective(Lower(model), Max,
        2x[2] - x[5] + 3y[1] -y[2] -4y[3])
    @constraint(Lower(model), [i=1:3], y[i] >= 0)
    @constraint(Lower(model), -  x[1] +0.2x[2]                + x[5] +  2x[6] - 4y[1] + 2y[2] + y[3] <= 12)
    @constraint(Lower(model),    x[1]          + x[3] - 2x[4]                         - 4y[2] + y[3] <= 10)
    @constraint(Lower(model),   5x[1]                 +  x[4]        +3.2x[6] + 2y[1] + 2y[2]        <= 15)
    @constraint(Lower(model),         -  3x[2]        -  x[4] + x[5]          - 2y[1]                <= 12)
    @constraint(Lower(model), - 2x[1] -   x[2]                                        -  y[2] + y[3] <= -2)
    @constraint(Lower(model),                                                 -  y[1] - 2y[2] - y[3] <= -2)
    @constraint(Lower(model),         -  2x[2] - 3x[3]        - x[5]                                 <= -3)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value.(x) ≈ [0, 4, 0, 15, 9.2, 0] atol=atol
    @test value.(y) ≈ [0, 0, 2] atol=atol
end

#=
    Test Problem Construction for Linear Bilevel Programming Problems 
    K. Moshirvaziri et al.
    Journal of Global Optimization, 1996
=#
function jump_SemiRand(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x[i=1:4])
    @variable(Lower(model), y[i=1:2])

    @objective(Upper(model), Min,
        -4x[1] +8x[2] +x[3] -x[4] + 9y[1] -9y[2])
    @constraint(Upper(model), [i=1:4], x[i] >= 0)
    @constraint(Upper(model), -9x[1] + 3x[2] -8x[3] + 3x[4] +3y[1]        <= -1)
    @constraint(Upper(model), +4x[1] -10x[2] +3x[3] + 5x[4] +8y[1] +8y[2] <= 25)
    @constraint(Upper(model), +4x[1] - 2x[2] -2x[3] +10x[4] -5y[1] +8y[2] <= 21)
    @constraint(Upper(model), +9x[1] - 9x[2] +4x[3] - 3x[4] - y[1] -9y[2] <= -1)
    @constraint(Upper(model), -2x[1] - 2x[2] +8x[3] - 5x[4] +5y[1] +8y[2] <= 20)
    @constraint(Upper(model), +7x[1] + 2x[2] -5x[3] + 4x[4] -5y[1]        <= 11)

    @constraint(Upper(model), [i=1:2], y[i] >= 0)
    
    @objective(Lower(model), Min,
        -9y[1] + 9y[2])
    @constraint(Lower(model), [i=1:2], y[i] >= 0)
    @constraint(Lower(model), -6x[1] + x[2] + x[3] - 3x[4] -9y[1] -7y[2] <= -15)
    @constraint(Lower(model),        +4x[2] +5x[3] +10x[4]               <= 26)
    @constraint(Lower(model), -9x[1] +9x[2] -9x[3] + 5x[4] -5y[1] -4y[2] <= -5)
    @constraint(Lower(model), +5x[1] +3x[2] + x[3] + 9x[4] + y[1] +5y[2] <= 32)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value.(x) ≈ [1.57504, 0.83668, 0.188807, 2.17092] atol=1e-4
    @test value.(y) ≈ [1.88765, 0.0] atol=1e-4
end

#=
    Decomposition Techniques in Mathematical Programming
    A. Conejo et al.
    Springer
=#

# Chapter 7.2, pag 281
function jump_DTMP_01(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -x + 4y)
    @constraint(Upper(model), y + 2x <= 8)

    @objective(Lower(model), Min, -x - y)
    @constraint(Lower(model), -y <= 0)
    @constraint(Lower(model), x + y <= 7)
    @constraint(Lower(model), -x <= 0)
    @constraint(Lower(model),  x <= 4)

    optimize!(model)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 1 atol=atol
    @test value(y) ≈ 6 atol=atol
end

#=
    modification to test variables used in a single level
=#

function jump_DTMP_01_mod1(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(UpperOnly(model), z)
    @variable(Lower(model), y)
    @variable(LowerOnly(model), w)

    @objective(Upper(model), Min, -x + 4y + z)
    @constraint(Upper(model), y + 2x + z <= 9)
    @constraint(Upper(model), z == 1)

    @objective(Lower(model), Min, -x - y + w)
    @constraint(Lower(model),  y >= 0)
    @constraint(Lower(model), x + y + w <= 8)
    @constraint(Lower(model),  x >= 0)
    @constraint(Lower(model),  x <= 4)
    @constraint(Lower(model),  w == 1)

    optimize!(model)

    @test value(x) ≈ 1 atol=atol
    @test value(y) ≈ 6 atol=atol
    @test value(z) ≈ 1 atol=atol
    @test value(w) ≈ 1 atol=atol
end

function jump_DTMP_01_mod2_error(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(UpperOnly(model), z)
    @variable(Lower(model), y)
    @variable(LowerOnly(model), w)

    @test_throws ErrorException @objective(Upper(model), Min, -x + 4y + z + w)
    @constraint(Upper(model), y + 2x + z <= 9)
    @constraint(Upper(model), z == 1)

    @test_throws ErrorException @objective(Lower(model), Min, -x - y + z)
end

jump_DTMP_01_mod3vec(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config()) = _jump_DTMP_01_mod3(optimizer, true, mode, config)
jump_DTMP_01_mod3(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config()) = _jump_DTMP_01_mod3(optimizer, false, mode, config)
function _jump_DTMP_01_mod3(optimizer, vectorized::Bool, mode, config)

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    if vectorized
        @variables(Upper(model), begin
            x
            z
        end)
    else
        @variable(Upper(model), x)
        @variable(Upper(model), z)
    end
    if vectorized
        @variables(Lower(model), begin
            y
            w
        end)
    else
        @variable(Lower(model), y)
        @variable(Lower(model), w)
    end

    @objective(Upper(model), Min, -x + 4y + z)
    if vectorized
        @constraints(Upper(model), begin
            y + 2x + z <= 9
            z == 1
        end)
    else
        @constraint(Upper(model), y + 2x + z <= 9)
        @constraint(Upper(model), z == 1)
    end

    @objective(Lower(model), Min, -x - y + w)
    if vectorized
        @constraints(Lower(model), begin
            y >= 0
            x + y + w <= 8
            x >= 0
            x <= 4
            w == 1
        end)
    else
        @constraint(Lower(model),  y >= 0)
        @constraint(Lower(model), x + y + w <= 8)
        @constraint(Lower(model),  x >= 0)
        @constraint(Lower(model),  x <= 4)
        @constraint(Lower(model),  w == 1)
    end

    optimize!(model)

    @test value(x) ≈ 1 atol=atol
    @test value(y) ≈ 6 atol=atol
    @test value(z) ≈ 1 atol=atol
    @test value(w) ≈ 1 atol=atol
end

#=
    Conejo, A. J., Baringo, L., Kazempour, S. J., and Siddiqui, A. S. (2016).
    Investment in Electricity Generation and Transmission. Springer.
=#
function jump_conejo2016(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config(); bounds = false)

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)
    BilevelJuMP.set_copy_names(model)

    @variable(Upper(model), x, start = 50)
    if bounds
        @variable(Lower(model), -1 <= y[i=1:3] <= 300, start = [50, 150, 0][i])
    else
        @variable(Lower(model), y[i=1:3], start = [50, 150, 0][i])
    end
    # 2 and 3 are lower only

    @constraint(Upper(model), lb0, x >= 0)
    @constraint(Upper(model), ub0, x <= 250)

    @objective(Lower(model), Min, 10y[1] + 12y[2] + 15y[3])
    @constraint(Lower(model), b, y[1] + y[2] + y[3] == 200)
    @constraint(Lower(model), ub1, y[1] <= x)
    @constraint(Lower(model), ub2, y[2] <= 150)
    @constraint(Lower(model), ub3, y[3] <= 100)
    @constraint(Lower(model), lb[i=1:3], y[i] >= 0)
    if bounds
        @variable(Upper(model), 0 <= lambda <= 20, DualOf(b), start = 15)
    else
        @variable(Upper(model), lambda, DualOf(b), start = 15)
    end

    @objective(Upper(model), Min, 40_000x + 8760*(10y[1]-lambda*y[1]))

    optimize!(model)

    primal_status(model)
    termination_status(model)

    @test objective_value(model) ≈ -190_000 atol=1e-1 rtol=1e-2
    @test value(x) ≈ 50 atol=1e-3 rtol=1e-2
    @test value.(y) ≈ [50, 150, 0] atol=1e-3 rtol=1e-2
    @test value(lambda) ≈ 15 atol=1e-3 rtol=1e-2
end

#=
    Bruno Fanzeres PhD thesis Robust Strategic Bidding in Auction-Based Markets.
=#
function jump_fanzeres2017(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), q1, start = 20)
    @variable(Lower(model), g[i=1:4], start = [20, 40, 40, 0][i])

    @constraint(Upper(model), q1 >= 0)
    @constraint(Upper(model), q1 <= 100)

    @objective(Lower(model), Min, 50g[2] + 100g[3] + 1000g[4])
    @constraint(Lower(model), b, g[1] + g[2] + g[3] + g[4] == 100)
    @constraint(Lower(model), g[1] <= q1)
    @constraint(Lower(model), g[2] <= 40)
    @constraint(Lower(model), g[3] <= 40)
    @constraint(Lower(model), g[4] <= 100)
    @constraint(Lower(model), lb[i=1:4], g[i] >= 0)

    @variable(Upper(model), lambda >= 0, DualOf(b), start = 1_000)

    @objective(Upper(model), Max, lambda*g[1])

    optimize!(model)

    primal_status(model)
    termination_status(model)

    @test objective_value(model) ≈ 20_000  atol=1e-1
    @test BilevelJuMP.lower_objective_value(model) ≈ 6_000  atol=1e-1
    @test value(q1) ≈ 20 atol=1e-3
    @test value.(g) ≈ [20, 40, 40, 0] atol=1e-3
    @test value(lambda) ≈ 1_000 atol=1e-3
end

function jump_eq_price(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    jger = 1
    G = [10, 15, 12]
    j = length(G)
    d = 26
    c = [1, 2, 1.5]
    p = 1
    q = 10

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), u[i=1:jger] >= 0, start = [2.0][i])

    @variable(Lower(model), def >= 0, start = 0)
    @variable(Lower(model), g[i=1:j] >= 0, start = [10.0, 4.0, 12.0][i])
    
    @constraint(Lower(model), MaxGen[i=1:j], g[i] <= G[i])
    @constraint(Lower(model), DemBal, sum(g[i] for i in 1:j) + def == d)
    @objective(Lower(model), Min, sum(u[i]*g[i] for i in 1:jger) +
                    sum(c[i+jger]*g[i+jger] for i in 1:(j-jger)) + 100*def)

    @variable(Upper(model), lambda >= 0, DualOf(DemBal), start = 2.0)
    @objective(Upper(model), Max, lambda*sum(g[i] for i in 1:jger) -
                        sum(c[i]*g[i] for i in 1:jger) - (lambda - p)*q)

    optimize!(model)

    primal_status(model)
    termination_status(model)

    @test value.(g) ≈ [10.0, 4.0, 12.0]  atol=atol
    @test value(def) ≈ 0.0  atol=atol
    @test value(lambda) ≈ 2.0  atol=atol
    @test -atol < value(u[1]) < 2.0  + atol

end

function jump_16(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

    atol = config.atol
    start = config.start_value

    MOI.empty!(optimizer)
    m = BilevelModel(()->optimizer, mode = mode)
    BilevelJuMP.set_copy_names(m)

    @variable(Upper(m), 0 <= x <= 3 )
    @variable(Lower(m), 0 <= y[1:2])
    @variable(Lower(m), 0 <= z[1:2])

    a = JuMP.Containers.DenseAxisArray([y[i]+ z[i] for i = 1:2], 1:2)
    d = JuMP.Containers.SparseAxisArray(Dict((1, i) => y[i] for i in 1:2))

    c = y[1] + y[2]

    @constraint(Upper(m), x >=1)

    @constraint(Lower(m), y[1] >= 0.5)
    @constraint(Lower(m), y[2] >= 0.5)

    @objective(Upper(m), Min, x)

    @objective(Lower(m), Min, y[1]+z[2])

    optimize!(m)

    # TODO add tests
    _a = value.(a)
    _c = value.(c)
    _d = value.(d)

end

function jump_conic01(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config(); bounds = false)

    # MOI.set(optimizer, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)
    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)
    BilevelJuMP.set_copy_names(model)

    @variable(Upper(model), x[i=1:3])
    @variable(Lower(model), y[i=1:3])


    @constraint(Upper(model), soc_up, x in SecondOrderCone())
    @constraint(Lower(model), soc_lw, y in SecondOrderCone())
    if bounds
        BilevelJuMP.set_dual_upper_bound_hint(soc_lw, +[5., 5., 5.])
        BilevelJuMP.set_dual_lower_bound_hint(soc_lw, -[5., 5., 5.])
        # bounds defined in the upper level are not dualized
        for i in 1:3
            @constraint(Upper(model), y[i] in MOI.LessThan(+5.0))
            @constraint(Upper(model), y[i] in MOI.GreaterThan(-5.0))
        end
    end

    if typeof(mode) <: BilevelJuMP.MixedMode
        BilevelJuMP.set_mode(soc_lw, BilevelJuMP.ProductMode())
    end

    @objective(Upper(model), Min, x[1])
    @objective(Lower(model), Min, y[1])

    optimize!(model)

    primal_status(model)
    termination_status(model)

    @test objective_value(model) ≈ 0  atol=1e-1
    @test value.(x) ≈ [0, 0, 0] atol=1e-3
    @test value.(y) ≈ [0, 0, 0] atol=1e-3
end

#=
    The models of bilevel programming with lower level second-order cone programs
    Chi et al. Journal of Inequalities and Applications 2014, 2014:168
    https://journalofinequalitiesandapplications.springeropen.com/articles/10.1186/1029-242X-2014-168
    https://core.ac.uk/download/pdf/81261904.pdf
=#

function jump_conic02(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config(); bounds = false)

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x, start = 6)
    vals = [2, 0]
    @variable(Lower(model), y[i=1:2], start = vals[i])

    @objective(Upper(model), Min, x + 3(y[1] -y[2]))
    if bounds
        @constraint(Upper(model), x in MOI.LessThan(+6.0))
        @constraint(Upper(model), x in MOI.GreaterThan(+2.0))
    else
        @constraint(Upper(model), x >= 2)
        @constraint(Upper(model), x <= 6)
    end
    
    @objective(Lower(model), Min, - (y[1] - y[2]))
    @constraint(Lower(model), lb_y_1, y[1] >= 0)
    @constraint(Lower(model), lb_y_2, y[2] >= 0)
    @constraint(Lower(model), con1, x +  (y[1] - y[2]) <=  8)
    @constraint(Lower(model), con2, x + 4(y[1] - y[2]) >=  8)
    @constraint(Lower(model), con3, x + 2(y[1] - y[2]) <= 12)
    @constraint(Lower(model), soc_lw, y in SecondOrderCone())

    if typeof(mode) <: BilevelJuMP.MixedMode
        BilevelJuMP.set_mode(soc_lw, BilevelJuMP.ProductMode())
    end

    if bounds
        BilevelJuMP.set_dual_upper_bound_hint(soc_lw, +[5., 5.])
        BilevelJuMP.set_dual_lower_bound_hint(soc_lw, -[5., 5.])
        # require lower bounds
        for con in [con1, con3]
            BilevelJuMP.set_dual_lower_bound_hint(con, -15)
        end
        # require upper bounds
        for con in [lb_y_1, lb_y_2, con2]
            BilevelJuMP.set_dual_upper_bound_hint(con, +15)
        end
        # bounds defined in the upper level are not dualized
        for i in 1:2
            @constraint(Upper(model), y[i] in MOI.LessThan(+5.0))
            @constraint(Upper(model), y[i] in MOI.GreaterThan(-5.0))
        end
    end

    unset_silent(model)
    optimize!(model)
    set_silent(model)

    @show primal_status(model)
    @show termination_status(model)
    @show value.(y)

    @test objective_value(model) ≈ 12  atol=1e-1
    @test value(x) ≈ 6 atol=1e-3
    @test value(y[2]) >= 0 - 1e-3
    @test value(y[1]) - value(y[2]) ≈ 2 atol=1e-3
    return
end
function jump_conic03(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config(); bounds = false)

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y[i=1:2])

    @objective(Upper(model), Min, x + 2(y[1] + y[2]))
    @constraint(Upper(model), y[1] + y[2] <= 3) # creates disconnected region
    if bounds
        @constraint(Upper(model), x in MOI.LessThan(+6.0))
        @constraint(Upper(model), x in MOI.GreaterThan(+0.0))
    else
        @constraint(Upper(model), x >= 0)
        @constraint(Upper(model), x <= 6)
    end

    @objective(Lower(model), Min, - y[1] - y[2])
    @constraint(Lower(model), con1, y[1] >= 0)
    @constraint(Lower(model), con2, y[2] <= 0)
    @constraint(Lower(model), con3, x +  (y[1] + y[2]) <= 8)
    @constraint(Lower(model), con4, x + 3(y[1] + y[2]) >= 8)
    @constraint(Lower(model), con5,-x +  (y[1] + y[2]) <= 0)
    @constraint(Lower(model), soc_lw, y in SecondOrderCone())

    if typeof(mode) <: BilevelJuMP.MixedMode
        BilevelJuMP.set_mode(soc_lw, BilevelJuMP.ProductMode())
    end

    if bounds
        BilevelJuMP.set_dual_upper_bound_hint(soc_lw, +[5., 5.])
        BilevelJuMP.set_dual_lower_bound_hint(soc_lw, -[5., 5.])
        # require lower bounds
        for con in [con2, con3, con5]
            BilevelJuMP.set_dual_lower_bound_hint(con, -15)
        end
        # require upper bounds
        for con in [con1, con4]
            BilevelJuMP.set_dual_upper_bound_hint(con, +15)
        end
        # bounds defined in the upper level are not dualized
        for i in 1:2
            @constraint(Upper(model), y[i] in MOI.LessThan(+5.0))
            @constraint(Upper(model), y[i] in MOI.GreaterThan(-5.0))
        end
    end

    optimize!(model)

    JuMP.raw_status(model)
    primal_status(model)
    termination_status(model)
    value.(y)

    @test objective_value(model) ≈ 6  atol=1e-1
    @test value(x) ≈ 2 atol=1e-3
    @test value(y[2]) <= 0 + 1e-3
    @test value(y[1]) + value(y[2]) ≈ 2 atol=1e-3
end
function jump_conic04(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config(); bounds = false)

    MOI.empty!(optimizer)
    model = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(model), x)
    @variable(Lower(model), y[i=1:3])

    @objective(Upper(model), Min, x + 3y[1])
    if bounds
        @constraint(Upper(model), x in MOI.GreaterThan(+2.0))
        @constraint(Upper(model), x in MOI.LessThan(+6.0))
    else
        @constraint(Upper(model), x >= 2)
        @constraint(Upper(model), x <= 6)
    end

    @objective(Lower(model), Min, - y[1])
    @constraint(Lower(model), a, x +  y[1] <=  8)
    @constraint(Lower(model), b, x + 4y[1] >=  8)
    @constraint(Lower(model), c, x + 2y[1] <= 12)
    # @constraint(Lower(model), d, y[1] >= 0)
    @constraint(Lower(model), soc_lw, y in SecondOrderCone())

    if typeof(mode) <: BilevelJuMP.MixedMode
        BilevelJuMP.set_mode(soc_lw, BilevelJuMP.ProductMode())
    end

    if bounds
        BilevelJuMP.set_dual_upper_bound_hint(soc_lw, +[5., 5., 5.])
        BilevelJuMP.set_dual_lower_bound_hint(soc_lw, -[5., 5., 5.])
        # require lower bounds
        for con in [a, c]
            BilevelJuMP.set_dual_lower_bound_hint(con, -15)
        end
        # require upper bounds
        for con in [b]
            BilevelJuMP.set_dual_upper_bound_hint(con, +15)
        end
        # bounds defined in the upper level are not dualized
        for i in 1:3
            @constraint(Upper(model), y[i] in MOI.LessThan(+5.0))
            @constraint(Upper(model), y[i] in MOI.GreaterThan(-5.0))
        end
    end

    optimize!(model)

    JuMP.raw_status(model)
    primal_status(model)
    termination_status(model)
    value.(y)

    @test objective_value(model) ≈ 12 atol=1e-1
    @test value(x) ≈ 6 atol=1e-3
    @test value(y[1]) ≈ 2 atol=1e-3
    @test sqrt(value(y[2])^2 + value(y[3])^2) <= 2 + 1e-3
end

# from: https://github.com/joaquimg/BilevelJuMP.jl/issues/82
function jump_fruits(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config(), p_max = 0.1)

    MOI.empty!(optimizer)
    m = BilevelModel(()->optimizer, mode = mode)

    @variable(Upper(m), 0 <= p <= p_max)
    @variable(Lower(m), 0 <= p_N[1:2] <= 10)
    @variable(Lower(m), 0 <= m_P[1:2] <= 10)
    @variable(Lower(m), 0 <= m_N[1:2] <= 10)
    
    @constraint(Upper(m), con_a, sum(m_P[i] - m_N[i] for i in 1:2) == 0)
    @objective(Upper(m), Min, -sum(0.01*m_N[i] + 0.003*m_P[i] for i in 1:2))
    
    @constraint(Lower(m), con_b[i=1:2], - p_N[i] + m_P[i] - m_N[i] == - 3*i)
    @objective(Lower(m), Min, -sum( - p_N[i]*0.1 - m_N[i]*(p + 0.01) + m_P[i]*(p - 0.003) for i in 1:2))

    BilevelJuMP.set_copy_names(m)
    optimize!(m, bilevel_prob = "pb.lp", solver_prob = "ps.lp",
        upper_prob = "pu.lp")
    @test isfile("pb.lp")
    @test isfile("ps.lp")
    @test isfile("pu.lp")
    optimize!(m, bilevel_prob = "pb.mof.json", solver_prob = "ps.mof.json",
        lower_prob = "pl.mof.json", upper_prob = "pu.mof.json")
    @test isfile("pb.mof.json")
    @test isfile("ps.mof.json")
    @test isfile("pl.mof.json")
    @test isfile("pu.mof.json")

    JuMP.raw_status(m)
    primal_status(m)
    if p_max < 0.09
        @test termination_status(m) in [MOI.INFEASIBLE, MOI.LOCALLY_INFEASIBLE]
    else
        @test termination_status(m) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]
        @test value.(m_P) ≈ [0, 0] atol=1e-3
        @test value.(m_N) ≈ [0, 0] atol=1e-3
        @test value.(p_N) ≈ [3, 6] atol=1e-3
    end
    return nothing
end

function jump_01_mixed(optimizer, config = Config())

    atol = config.atol

    # config.bound_hint = true

    # min -4x -3y
    # s.t.
    # y = argmin_y y
    #      2x + y <= 4
    #       x +2y <= 4
    #       x     >= 0
    #           y >= 0
    #
    # sol: x = 2, y = 0
    # obj_upper = -8
    # obj_lower =  0

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(
        ()->optimizer,
        mode = BilevelJuMP.MixedMode(
            default = BilevelJuMP.FortunyAmatMcCarlMode()))
    BilevelJuMP.set_copy_names(model)

    @variable(Upper(model), x >= 0)
    @variable(Lower(model), y >= 0)

    @objective(Upper(model), Min, -4x -3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c1, 2x+y <= 4
        c2, x+2y <= 4
    end)

    @test_throws ErrorException BilevelJuMP.set_mode(c1, BilevelJuMP.MixedMode())
    @test_throws ErrorException BilevelJuMP.set_mode(c1, BilevelJuMP.StrongDualityMode())
    @test_throws ErrorException BilevelJuMP.set_mode(x, BilevelJuMP.MixedMode())
    @test_throws ErrorException BilevelJuMP.set_mode(x, BilevelJuMP.StrongDualityMode())

    BilevelJuMP.set_mode(x, BilevelJuMP.SOS1Mode())
    BilevelJuMP.set_mode(y, BilevelJuMP.IndicatorMode())
    BilevelJuMP.set_mode(c1, BilevelJuMP.SOS1Mode())
    BilevelJuMP.set_mode(c2, BilevelJuMP.IndicatorMode())

    # if config.bound_hint
    #     for cref in [c1, c2, c3, c4]
    #         BilevelJuMP.set_dual_upper_bound_hint(cref, 10)
    #         BilevelJuMP.set_dual_lower_bound_hint(cref, -10)
    #     end
    #     BilevelJuMP.set_primal_lower_bound_hint(x, -1)
    #     BilevelJuMP.set_primal_lower_bound_hint(y, -1)
    #     BilevelJuMP.set_primal_upper_bound_hint(x, 5)
    #     BilevelJuMP.set_primal_upper_bound_hint(y, 5)
    # end

    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED]

    @test objective_value(model) ≈ -8 atol=atol

    @test value(x) ≈  2 atol=atol
    @test value(y) ≈  0 atol=atol

    # @test dual(c1) ≈ [0] atol=atol #NLP fail
    @test dual(c2) ≈ [0] atol=atol
    # @test dual(c3) ≈ [0] atol=atol
    # @test dual(c4) ≈ [1] atol=atol #NLP fail

end

function jump_01_sum_agg(optimizer, config = Config())

    atol = config.atol

    # config.bound_hint = true

    # min -4x -3y
    # s.t.
    # y = argmin_y y
    #      2x + y <= 4
    #       x +2y <= 4
    #       x     >= 0
    #           y >= 0
    #      2x + y <= 8 (loose)
    #       x +2y <= 8 (loose)
    #
    # sol: x = 2, y = 0
    # obj_upper = -8
    # obj_lower =  0

    atol = config.atol

    MOI.empty!(optimizer)
    model = BilevelModel(
        ()->optimizer,
        mode = BilevelJuMP.MixedMode(
            default = BilevelJuMP.ProductMode(1e-9, aggregation_group = 1)))
    BilevelJuMP.set_copy_names(model)

    @variable(Upper(model), x >= 0)
    @variable(Lower(model), y >= 0)

    @objective(Upper(model), Min, -4x -3y)

    @objective(Lower(model), Min, y)

    @constraints(Lower(model), begin
        c1a, 2x+y <= 4
        c2a, x+2y <= 4
        c1b, 2x+y <= 8
        c2b, x+2y <= 8
    end)

    BilevelJuMP.set_mode(c1a, BilevelJuMP.ProductMode(1e-9, aggregation_group = 2))
    BilevelJuMP.set_mode(c1b, BilevelJuMP.ProductMode(1e-9, aggregation_group = 2))
    BilevelJuMP.set_mode(c2a, BilevelJuMP.ProductMode(1e-9, aggregation_group = 3))
    BilevelJuMP.set_mode(c2b, BilevelJuMP.ProductMode(1e-9, aggregation_group = 3))

    if config.bound_hint
        # for cref in [c1, c2, c3, c4]
        #     BilevelJuMP.set_dual_upper_bound_hint(cref, 10)
        #     BilevelJuMP.set_dual_lower_bound_hint(cref, -10)
        # end
        BilevelJuMP.set_primal_lower_bound_hint(x, -1)
        BilevelJuMP.set_primal_lower_bound_hint(y, -1)
        BilevelJuMP.set_primal_upper_bound_hint(x, 5)
        BilevelJuMP.set_primal_upper_bound_hint(y, 5)
    end

    optimize!(model)

    primal_status(model)

    @test termination_status(model) in [MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ALMOST_LOCALLY_SOLVED]

    @test objective_value(model) ≈ -8 atol=atol

    @test value(x) ≈  2 atol=atol
    @test value(y) ≈  0 atol=atol

    # @test dual(c1) ≈ [0] atol=atol #NLP fail
    @test dual(c2a) ≈ [0] atol=atol
    # @test dual(c3) ≈ [0] atol=atol
    # @test dual(c4) ≈ [1] atol=atol #NLP fail

end

"""
From: https://github.com/joaquimg/BilevelJuMP.jl/issues/182
By: LukasBarner

Siddiqui S, Gabriel SA (2013). An sos1-based approach for solving mpecs with a natural gas market applica-
tion. Networks and Spatial Economics 13(2):205–227.
"""
function jump_qp_lower_min(config = Config())
    F = [1,2]
    c = Dict(1=>1, 2=>1)
    C = 1
    a = 13
    b = 1

    model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode())

    @variable(Lower(model), q[F] >= 0)
    @variable(Upper(model), Q >= 0)

    @objective(Upper(model), Max, ((a-b * (q[1] + q[2] + Q)) * Q - C*Q) )

    @objective(Lower(model), Min, -((a-b * (q[1] + q[2] + Q)) * q[1] - C*q[1] + (a-b * (q[1] + q[2] + Q)) * q[2] - C*q[2] + b*q[1]*q[2]) )

    optimize!(model)

    @test isapprox(value.(Q), 6; atol=1e-5)
    @test isapprox(value.(q).data, [2,2]; atol=1e-5)
end
function jump_qp_lower_max(config = Config())
    F = [1,2]
    c = Dict(1=>1, 2=>1)
    C = 1
    a = 13
    b = 1

    model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode())

    @variable(Lower(model), q[F] >= 0)
    @variable(Upper(model), Q >= 0)

    @objective(Upper(model), Max, ((a-b * (q[1] + q[2] + Q)) * Q - C*Q) )

    @objective(Lower(model), Max, +((a-b * (q[1] + q[2] + Q)) * q[1] - C*q[1] + (a-b * (q[1] + q[2] + Q)) * q[2] - C*q[2] + b*q[1]*q[2]) )

    optimize!(model)

    @test isapprox(value.(Q), 6; atol=1e-5)
    @test isapprox(value.(q).data, [2,2]; atol=1e-5)
end