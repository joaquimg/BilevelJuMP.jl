
function jump_01(optimizer, mode = BilevelJuMP.SOS1Mode())

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

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @objective(Lower(model), Min, y)

    @constraint(Lower(model), 2x+y <= 4)
    @constraint(Lower(model), x+2y <= 4)
    @constraint(Lower(model), x >= 0)
    @constraint(Lower(model), y >= 0)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ -8

    @test value(x) ≈  2
    @test value(y) ≈  0

end

function jump_02(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, x + 3y)

    @objective(Lower(model), Min, -y)

    @constraint(Lower(model), x+y <= 8)
    @constraint(Lower(model), x+4y >= 8)
    @constraint(Lower(model), x+2y <= 13)
    @constraint(Lower(model), x >= 1)
    @constraint(Lower(model), x <= 6)
    @constraint(Lower(model), y >= 0)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ 12

    @test value(x) ≈ 6
    @test value(y) ≈ 2

end


#=
    From:
    Foundations of Bilevel Programming
    by: Stephan Dempe
    in: Nonconvex Optimization and Its Applications
=#

# obs: example 2 is from the book

# Cap 3.2, Pag 25
function jump_03(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Lower(model), x)
    @variable(Upper(model), y)

    @objective(Upper(model), Min, 3x + y)
    @constraint(Upper(model), x <= 5)
    @constraint(Upper(model), y <= 8)
    @constraint(Upper(model), y >= 0)

    @objective(Lower(model), Min, -x)

    @constraint(Lower(model),  x +  y <= 8)
    @constraint(Lower(model), 4x +  y >= 8)
    @constraint(Lower(model), 2x +  y <= 13)
    @constraint(Lower(model), 2x - 7y <= 0)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ 3* (3.5*8/15) + (8/15)

    @test value(x) ≈ 3.5*8/15
    @test value(y) ≈ 8/15

end
# change the bound on x to lower level
function jump_04(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Lower(model), x)
    @variable(Upper(model), y)

    @objective(Upper(model), Min, 3x + y)
    @constraint(Upper(model), y <= 8)
    @constraint(Upper(model), y >= 0)
    
    @objective(Lower(model), Min, -x)
    
    @constraint(Lower(model),  x +  y <= 8)
    @constraint(Lower(model), 4x +  y >= 8)
    @constraint(Lower(model), 2x +  y <= 13)
    @constraint(Lower(model), 2x - 7y <= 0)
    @constraint(Lower(model), x <= 5)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ 3* (3.5*8/15) + (8/15)

    @test value(x) ≈ 3.5*8/15
    @test value(y) ≈ 8/15

end

# Sec 3.3 , pag 30 -> product of x and y in lower level objective

# Sec 3.4.1 , pag 32
function jump_05(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ 0

    @test value(x[1]) ≈ 1/100
    @test value(x[2]) ≈ 0
    @test value(y) ≈ 0

end

# sec 3.5.2.2 pag 44 -> product

# sec 3.7 pag 59
function jump_3SAT(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    n = 7 # maximum literals
    clauses = [[1,2,3],[-1,-4,3],[7,-6,4],[5,6,7]]

    @variable(Lower(model), x[i=1:n])
    @variable(Upper(model), ya[i=1:n])
    @variable(Upper(model), yb[i=1:n])
    @variable(Upper(model), z)
    # @variable(Upper(model), z)

    @objective(Upper(model), Min, sum(x[i] for i in 1:n) - z)
    @constraint(Upper(model), z <= 1)
    @constraint(Upper(model), z >= 0)
    @constraint(Upper(model), [i=1:n], ya[i] >= 0)
    @constraint(Upper(model), [i=1:n], ya[i] <= 1)
    @constraint(Upper(model), [i=1:n], yb[i] >= 0)
    @constraint(Upper(model), [i=1:n], yb[i] <= 1)
    @constraint(Upper(model), [i=1:n], ya[i] + yb[i] == 1)
    for c in clauses
        @constraint(Upper(model),
            sum(i > 0 ? ya[i] : yb[-i] for i in c) >= z)
    end

    @objective(Lower(model), Min, -sum(x[i] for i in 1:n))

    @constraint(Lower(model), [i=1:n], x[i] >= 0)
    @constraint(Lower(model), [i=1:n], x[i] <= ya[i])
    @constraint(Lower(model), [i=1:n], x[i] <= yb[i])

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ -1

    # 3SAT is yese IFF obj = -1

end

# sec 5.1 pag 121 -> product

# sec 5.1 pag 127
function jump_quad_01_a(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Lower(model), x)
    @variable(Upper(model), y)

    @objective(Upper(model), Min, x^2 + y)
    @constraint(Upper(model), -x -y <= 0)

    @objective(Lower(model), Min, x)
    @constraint(Lower(model), x >= 0)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ 0
    @test value(x) ≈ 0
    @test value(y) ≈ 0

end
function jump_quad_01_b(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Lower(model), x)
    @variable(Upper(model), y)

    @objective(Upper(model), Min, x^2 + y)
    
    @objective(Lower(model), Min, x)
    @constraint(Lower(model), -x -y <= 0)
    @constraint(Lower(model), x >= 0)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ 0.5^2 - 0.5
    @test value(x) ≈ 0.5
    @test value(y) ≈ -0.5

end
function jump_quad_01_c(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Lower(model), x)
    @variable(Upper(model), y)

    @objective(Upper(model), Min, x^2 + y)
    @constraint(Upper(model), -x -y <= 0)
    @constraint(Upper(model), x >= 0)
    
    @objective(Lower(model), Min, x)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

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
function jump_int_01(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)


end

# sec 8.1 pag 257
function jump_int_02(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

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
function jump_06(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, x - 4y)
    @constraint(Upper(model), x >= 0)
    
    @objective(Lower(model), Min, y)

    @constraint(Lower(model), -x  - y <= -3)
    @constraint(Lower(model), -2x + y <= 0)
    @constraint(Lower(model), 2x  + y <= 12)
    @constraint(Lower(model), 3x + -2y <= 4) # signs are wrong in some versions of the book
    @constraint(Lower(model), y >= 0)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) ≈ -12

    @test value(x) ≈ 4
    @test value(y) ≈ 4

end

# pag 208 ex 5.3.1
function jump_07(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x[i=1:2])
    @variable(Lower(model), y[i=1:3])

    @objective(Upper(model), Min,
        -8x[1] -4x[2] + 4y[1] - 40y[2] - 4y[3])
    @constraint(Upper(model), [i=1:2], x[i] >= 0)
    
    @objective(Lower(model), Min,
        x[1]+ 2x[2] + y[1] + y[2] + 2y[3])
    @constraint(Lower(model), [i=1:3], y[i] >= 0)

    @constraint(Lower(model), -y[1] + y[2] + y[3] <= 1)
    @constraint(Lower(model), 2x[1] - y[1] + 2y[2] - 0.5y[3] <= 1)
    @constraint(Lower(model), 2x[2] +2y[1] - y[2] - 0.5y[3] <= 1)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value.(x) ≈ [0, 0.9]
    @test value.(y) ≈ [0, 0.6, 0.4]

end

# pag 208 ex 5.3.1
function jump_08(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), x >= 0)
    
    @objective(Lower(model), Min, -5x - y)
    @constraint(Lower(model), y >= 0)

    @constraint(Lower(model), -x - y/2 <= -2)
    @constraint(Lower(model), -x/4 + y <= 2)
    @constraint(Lower(model), x + y/2 <= 8)
    @constraint(Lower(model), x - 2y <= 4)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 8/9
    @test value(y) ≈ 20/9

end

# pag 271 ex 7.1.1 -> quadratic terms
# pag 281 ex 7.2.2 -> quadratic terms
# pag 302 ex 8.1.1 -> quadratic terms

# pag 304 ex 8.1.2
function jump_09a(optimizer, mode = BilevelJuMP.SOS1Mode())

    # degenerate second level
    # its case that show that the KKT approach is OPTIMISTIC

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y[i=1:2])

    @objective(Upper(model), Min, -10.1x + 10y[1] - y[2])
    @constraint(Upper(model), x >= 0)
    
    @objective(Lower(model), Min, -y[1] - y[2])
    @constraint(Lower(model), y[1] >= 0)
    @constraint(Lower(model), y[2] >= 0)

    @constraint(Lower(model), x - y[1] <= 1)
    @constraint(Lower(model), x + y[2] <= 1)
    @constraint(Lower(model), y[1] + y[2] <= 1)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 0
    @test value.(y) ≈ [0, 1]

end
function jump_09b(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 0
    @test value.(y) ≈ [0, 1]

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
function jump_10(optimizer, mode = BilevelJuMP.SOS1Mode())

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

    model = BilevelModel()

    @variable(Upper(model), b)
    @variable(Lower(model), x[i=1:n])

    @objective(Upper(model), Min, sum(c_up[i]*x[i] for i in 1:n) + f*b)
    @constraint(Upper(model), b >= b_l)
    @constraint(Upper(model), b <= b_u)

    @objective(Lower(model), Min, sum(c_lw[i]*x[i] for i in 1:n) )
    @constraint(Lower(model), [i=1:n], x[i] >= 0)
    @constraint(Lower(model), [i=1:n], x[i] <= 1)
    @constraint(Lower(model), sum(a_lw[i]*x[i] for i in 1:n) >= b)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    # @test value(x) ≈ 0
    # @test value(y) ≈ [1, 0]

end

# pag 21 ex 2.1
function jump_11a(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -x - 2y)
    @constraint(Upper(model), 2x - 3y >= -12)
    @constraint(Upper(model), x + y <= 14)
    
    @objective(Lower(model), Min, -y)
    @constraint(Lower(model), -3x + y <= -3)
    @constraint(Lower(model), 3x + y <= 30)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 8
    @test value(y) ≈ 6

end
function jump_11b(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -x - 2y)
    
    @objective(Lower(model), Min, -y)
    @constraint(Lower(model), 2x - 3y >= -12)
    @constraint(Lower(model), x + y <= 14)
    @constraint(Lower(model), -3x + y <= -3)
    @constraint(Lower(model), 3x + y <= 30)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 6
    @test value(y) ≈ 8

end

# pag 45 ex 3.3
function jump_12(optimizer, mode = BilevelJuMP.SOS1Mode())

    for a in [0 0.1 0.2]
        model = BilevelModel()

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

        optimize!(model, optimizer, mode)

        primal_status(model)

        termination_status(model)

        @test value(x) ≈ a/2
        @test value(y) ≈ 1 + a/2
        @test value(z) ≈ 1
    end
end

# pag 45 ex 3.3
function jump_13_quad(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, (x -1 )^2 + y^2)
    @constraint(Upper(model), x >= -2)
    @constraint(Upper(model), x <= +2)

    @objective(Lower(model), Min, -y)
    @constraint(Lower(model), x + y <= 2)
    @constraint(Lower(model), -x + y <= 2)
    @constraint(Lower(model), y >= 0)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 3/2
    @test value(y) ≈ 1/2
end

# pag 290 ex 8.2
function jump_14(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 0
    @test value(y) ≈ 10
    @test value(z) ≈ 5
end

# pag 300 ex 8.5.2
function jump_15a_INT(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 1
    @test value(y) ≈ 75
    @test value(Z) ≈ 21+2/3
end
function jump_15b_INT(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 1
    @test value(y) ≈ 75
    @test value(Z) ≈ 21+2/3
end

#=
    Princeton Handbook of Test Problems in Local and Global Optimization
    Floudas c., Pardalos P., et al.
    HTP
=#

# 9.2.2
function jump_HTP_lin01(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y[i=1:2])

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 5
    @test value.(y) ≈ [4, 2]
end

# 9.2.3
function jump_HTP_lin02(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -x - 3y)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), y >= 0)

    @objective(Lower(model), Min, y)
    @constraint(Lower(model), -y <= 0)
    @constraint(Lower(model), -x + y <= 3)
    @constraint(Lower(model),   x + 2y <= 12)
    @constraint(Lower(model),  4x -  y <= 12)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 4
    @test value(y) ≈ 4
end

# 9.2.4 - parg 211
function jump_HTP_lin03(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

    @variable(Upper(model), x[i=1:2])
    @variable(Lower(model), y[i=1:6])

    @objective(Upper(model), Min,
        4y[1] - 40y[2] -4y[3] -8x[1] -4x[2])
    @constraint(Upper(model), [i=1:2], x[i] >= 0)
    @constraint(Upper(model), [i=1:6], y[i] >= 0)

    H1 = [ -1  1  1   1  0  0;
           -1  2 -1/2 0  1  0;
            2 -1 -1/2 0  0  1
    ]
    H2 = [0 0;
          2 0;
          0 2
    ]

    b = [1 1 1]

    @objective(Lower(model), Min,
        y[1] + y[2] + 2y[3] +x[1] +2x[2])

    # TODO fix broadcast
    # @constraint(Lower(model), H1*y + H2*x .== b)
    @constraint(Lower(model), [i=1:6], y[i] >= 0)

    @constraint(Lower(model), -y[1] +  y[2] +       y[3]         +y[4] == 1)
    @constraint(Lower(model), -y[1] + 2y[2] - (1/2)*y[3] + 2x[1] +y[5] == 1)
    @constraint(Lower(model), 2y[1] -  y[2] - (1/2)*y[3] + 2x[2] +y[6] == 1)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value.(x) ≈ [0, 0.9]
    @test value.(y) ≈ [0, 0.6, 0.4, 0, 0, 0]
end

# 9.2.5
function jump_HTP_lin04(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, x - 4y)
    # @constraint(Upper(model), x >= 0)
    # @constraint(Upper(model), y >= 0)

    @objective(Lower(model), Min, y)
    @constraint(Lower(model), -2x +  y <= 0)
    @constraint(Lower(model),  2x + 5y <= 108)
    @constraint(Lower(model),  2x - 3y <= -4)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 19
    @test value(y) ≈ 14
end

# 9.2.6
function jump_HTP_lin05(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y[i=1:2])

    @objective(Upper(model), Min, -x + 10y[1] - y[2])
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    @objective(Lower(model), Min, -y[1] - y[2])
    @constraint(Lower(model),    x -  y[1] <= 1)
    @constraint(Lower(model),    x +  y[2] <= 1)
    @constraint(Lower(model), y[1] +  y[2] <= 1)
    @constraint(Lower(model),  -y[1] <= 0)
    @constraint(Lower(model),  -y[2] <= 0)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    # book solution looks incorrect
    # the actual solutions seems to be
    @test value(x) ≈ 0
    @test value.(y) ≈ [0, 1]
end

# 9.2.7
function jump_HTP_lin06(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    # book solution looks incorrect
    # the actual solutions seems to be
    @test value(x) ≈ 16
    @test value(y) ≈ 11
end

# 9.2.8 - parg 216
function jump_HTP_lin07(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

    @variable(Upper(model), x[i=1:2])
    @variable(Lower(model), y[i=1:3])

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value.(x) ≈ [0, 0.9]
    @test value.(y) ≈ [0, 0.6, 0.4]
end

# 9.2.9 - parg 217
function jump_HTP_lin08(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

    @variable(Upper(model), x[i=1:2])
    @variable(Lower(model), y[i=1:2])

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    # solution frm the book is incorrect
    # using solution from: https://www.gams.com/latest/emplib_ml/libhtml/emplib_flds918.html
    @test value.(x) ≈ [2, 0]
    @test value.(y) ≈ [1.5, 0]
end

# 9.2.10
function jump_HTP_lin09(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 8/9#0.888_888_888_888
    @test value(y) ≈ 20/9# 2.222_222_222_222
end

# 9.2.11 - parg 219
function jump_HTP_lin10(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

    @variable(Upper(model), x[i=1:2])
    @variable(Lower(model), y[i=1:2])

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    # solution frm the book is incorrect
    # using solution from: https://www.gams.com/latest/emplib_ml/libhtml/emplib_flds918.html
    @test value.(x) ≈ [2, 0]
    @test value.(y) ≈ [1.5, 0]
end

# TODO - add quadratic problems

# 9.3.2 - parg 221
function jump_HTP_quad01(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min,
        (x-5)^2 + (2y+1)^2)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), y >= 0) # only in lowrrin GAMS

    @objective(Lower(model), Min,
        (y-1)^2 -1.5*x*y)

    @constraint(Lower(model), -3x +    y <= -3)
    @constraint(Lower(model),   x - 0.5y <= 4)
    @constraint(Lower(model),   x +    y <= 7)
    # @constraint(Lower(model), y >= 0) # GAMS file has this constraint


    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 1
    @test value(y) ≈ 0
end

# 9.3.3- parg 222
function jump_HTP_quad02(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min,
        x^2 + (y-10)^2)
    @constraint(Upper(model), - x + y <= 0)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), x <= 15)
    @constraint(Upper(model), y >= 0)
    @constraint(Upper(model), y <= 20)

    @objective(Lower(model), Min,
        (x + 2y - 30)^2)

    @constraint(Lower(model),   x +    y <= 20)
    @constraint(Lower(model),     -    y <= 0)
    @constraint(Lower(model),          y <= 20)


    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 10
    @test value(y) ≈ 10
end

# 9.3.4- parg 223
function jump_HTP_quad03(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x[i=1:2])
    @variable(Lower(model), y[i=1:2])

    @objective(Upper(model), Min,
        2x[1] + 2x[2] -3y[1] - 3y[2] -60)
    @constraint(Upper(model), x[1] + x[2] + y[1] -2y[2] -40 <= 0)
    @constraint(Upper(model), [i=1:2], x[i] >= 0)
    @constraint(Upper(model), [i=1:2], x[i] <= 50)
    @constraint(Upper(model), [i=1:2], y[i] >= -10)
    @constraint(Upper(model), [i=1:2], y[i] <= 20)

    @objective(Lower(model), Min,
        (-x[1] + y[1] + 40)^2 + (-x[2] + y[2] + 20)^2)
        # the boo does not contain the 40, it is a 20 there
        # however the solution does not match
        # the file https://www.gams.com/latest/emplib_ml/libhtml/emplib_flds923.html
        # has a 40

    @constraint(Lower(model),  [i=1:2],- x[i] + 2y[i] <= -10)
    @constraint(Lower(model), [i=1:2], y[i] >= -10)
    @constraint(Lower(model), [i=1:2], y[i] <= 20)


    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ 0

    sol = vcat(value.(x), value.(y))
    @test sol ≈ [0 ; 0 ; -10; -10] || sol ≈ [0 ; 30; -10; 10]
    # gurobi found the second solution  which is actually feasible and
    # has the same objective value
end

# 9.3.5- parg 225
function jump_HTP_quad04(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y[i=1:2])

    @objective(Upper(model), Min,
        0.5*((y[1]-2)^2+(y[2]-2)^2) )
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    @objective(Lower(model), Min,
        0.5*(y[1]^2) + y[2])

    @constraint(Lower(model), y[1] + y[2] == x)
    @constraint(Lower(model), [i=1:2], y[i] >= 0)


    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ 0.5

    @test value(x) ≈ 3
    @test value.(y) ≈ [1 ; 2]
end

# 9.3.6- parg 226
function jump_HTP_quad05(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min,
        (x-3)^2+(y-2)^2 )
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), x <= 8)

    @objective(Lower(model), Min,
        (y-5)^2 )

    @constraint(Lower(model), -2x+y <= 1)
    @constraint(Lower(model),  x-2y <= 2)
    @constraint(Lower(model),  x+2y <= 14)
    @constraint(Lower(model), y >= 0)


    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ 5

    @test value(x) ≈ 1
    @test value(y) ≈ 3
end

# 9.3.7- parg 227
function jump_HTP_quad06(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x[i=1:2])
    @variable(Lower(model), y[i=1:2])

    @objective(Upper(model), Min,
        x[1]^2 -2x[1] +x[2]^2 - 2x[2] +y[1]^2 +y[2]^2)
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    @objective(Lower(model), Min,
        (-x[1] + y[1])^2 + (-x[2] + y[2])^2)

    @constraint(Lower(model), [i=1:2], y[i] >= 0.5)
    @constraint(Lower(model), [i=1:2], y[i] <= 1.5)


    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ -1

    sol = vcat(value.(x), value.(y))
    @test sol ≈ [0.5 ; 0.5; 0.5; 0.5]

end
function jump_HTP_quad06b(optimizer, mode = BilevelJuMP.SOS1Mode())
    # TODO reviews the behaviour comment
    model = BilevelModel()

    @variable(Upper(model), x[i=1:2])
    @variable(Lower(model), y[i=1:2])

    @objective(Upper(model), Min,
        x[1]^2 +x[2]^2 +y[1]^2 - 3y[1] +y[2]^2 - 3y[2])
    @constraint(Upper(model), [i=1:2], y[i] >= 0)

    @objective(Lower(model), Min,
        (-x[1] + y[1])^2 + (-x[2] + y[2])^2)

    @constraint(Lower(model), [i=1:2], y[i] >= 0.5)
    @constraint(Lower(model), [i=1:2], y[i] <= 1.5)


    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)
    termination_status(model)
    # @test objective_value(model) ≈ -1

    sol = vcat(value.(x), value.(y))
    @test sol ≈ [0.5 ; 0.5; 0.5; 0.5]*sqrt(3)

end

# 9.3.8- parg 228
function jump_HTP_quad07(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min,
        (x-5)^2 + (2y+1)^2)
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), y >= 0) # only in lowrrin GAMS

    @objective(Lower(model), Min,
        (y-1)^2 -1.5*x*y)

    @constraint(Lower(model), -3x +    y <= -3)
    @constraint(Lower(model),   x - 0.5y <= 4)
    @constraint(Lower(model),   x +    y <= 7)
    @constraint(Lower(model), y >= 0) # only difference from problem 1


    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 1
    @test value(y) ≈ 0
end

# 9.3.9 - parg 229
function jump_HTP_quad08(optimizer, mode = BilevelJuMP.SOS1Mode())
    # Q objective is not PSD

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min,
        -(4x-3)*y+(2x+1) )
    @constraint(Upper(model), x >= 0)
    @constraint(Upper(model), x <= 1)
    @constraint(Upper(model), y >= 0)
    @constraint(Upper(model), y <= 1)

    @objective(Lower(model), Min,
        -(1-4x)*y -(2x+2) )

    @constraint(Lower(model), y >= 0)
    @constraint(Lower(model), y <= 1)


    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ 1.5

    @test value(x) ≈ 0.25
    @test value(y) ≈ 0
end

# 9.3.10- parg 230
function jump_HTP_quad09(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y[i=1:2])

    @objective(Upper(model), Min,
        x+y[2] )
    @constraint(Upper(model), [i=1:2], y[i] >= 0)
    @constraint(Upper(model), x >= 0)

    @objective(Lower(model), Min,
        2y[1]+x*y[2])

    @constraint(Lower(model), x + 4 <= y[1] + y[2])
    # this 4 is missing from the book
    # see https://www.gams.com/latest/emplib_ml/libhtml/emplib_flds929.html
    @constraint(Lower(model), [i=1:2], y[i] >= 0)


    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)
    termination_status(model)
    @test objective_value(model) ≈ 2

    @test value(x) ≈ 2
    @test value.(y) ≈ [6 ; 0]
end

#=
    GAMS educational examples by Michael Ferris
=#

# from https://www.gams.com/latest/emplib_ml/libhtml/emplib_jointc1.html
function jump_jointc1(optimizer, mode = BilevelJuMP.SOS1Mode())

    ###
    ### PART 1
    ###

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, x)
    @constraint(Upper(model), x >= 1)
    @constraint(Upper(model), y >= +x)
    @constraint(Upper(model), y >= -x)

    # this lower problem leads to x == -x (infeasible because x >= 1)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), y >= -x)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    @test termination_status(model) in [MOI.INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED]

    ###
    ### PART 2
    ###

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, x)
    @constraint(Upper(model), x >= 1)
    @constraint(Upper(model), y >= +x)
    @constraint(Upper(model), y >= -x)

    # this lower problem leads to x == -x (infeasible because x >= 1)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), y >= -x)
    @constraint(Lower(model), y >= +x)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    @test termination_status(model) in [MOI.OPTIMAL]

    @test value(x) ≈ 1
    @test value(y) ≈ 1
end

# from https://www.gams.com/latest/emplib_ml/libhtml/emplib_jointc2.html
function jump_jointc2(optimizer, mode = BilevelJuMP.SOS1Mode())

    ###
    ### PART 1
    ###

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Max, x)

    @constraint(Upper(model), x >= 1)
    @constraint(Upper(model), x <= 2)
    @constraint(Upper(model), y <= 100)

    @constraint(Upper(model), y >= x - 2)
    @constraint(Upper(model), y >= -x)

    # this lower problem leads to x == -x (infeasible because x >= 1)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), y >= -x)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    @test termination_status(model) in [MOI.OPTIMAL]

    @test value(x) ≈ +1
    @test value(y) ≈ -1

    ###
    ### PART 2
    ###

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    @test termination_status(model) in [MOI.OPTIMAL]

    @test value(x) ≈ 2
    @test value(y) ≈ 0
end

#=
    An Efficient Point Algorithm for Two-Stage Optimization Problem
    J.F. Bard
    Operations Research, 1983
=#
function jump_EffPointAlgo(optimizer, mode = BilevelJuMP.SOS1Mode())
    # send y to upper level

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value.(x) ≈ [0, 4, 0, 15, 9.2, 0]
    @test value.(y) ≈ [0, 0, 2]
end

#=
    Test Problem Construction for Linear Bilevel Programming Problems 
    K. Moshirvaziri et al.
    Journal of Global Optimization, 1996
=#
function jump_SemiRand(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

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
function jump_DTMP_01(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @objective(Upper(model), Min, -x + 4y)
    @constraint(Upper(model), y + 2x <= 8)

    @objective(Lower(model), Min, -x - y)
    @constraint(Lower(model), -y <= 0)
    @constraint(Lower(model), x + y <= 7)
    @constraint(Lower(model), -x <= 0)
    @constraint(Lower(model),  x <= 4)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test value(x) ≈ 1
    @test value(y) ≈ 6
end

#=
    modification to test variables used in a single level
=#

function jump_DTMP_01_mod1(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

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

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    @test value(x) ≈ 1
    @test value(y) ≈ 6
    @test value(z) ≈ 1
    @test value(w) ≈ 1
end

function jump_DTMP_01_mod2_error(optimizer, mode = BilevelJuMP.SOS1Mode())

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(UpperOnly(model), z)
    @variable(Lower(model), y)
    @variable(LowerOnly(model), w)

    @test_throws ErrorException @objective(Upper(model), Min, -x + 4y + z + w)
    @constraint(Upper(model), y + 2x + z <= 9)
    @constraint(Upper(model), z == 1)

    @test_throws ErrorException @objective(Lower(model), Min, -x - y + z)
end