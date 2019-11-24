
function jump_01(optimizer, mode = BilevelJuMP.SOS1Mode)

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

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y)

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

    @test objective_value(model) == -8

    @test value(x) ==  2
    @test value(y) ==  0

end

function jump_02(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y)

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

    @test objective_value(model) == 12

    @test value(x) == 6
    @test value(y) == 2

end


#=
    From:
    Foundations of Bilevel Programming
    by: Stephan Dempe
    in: Nonconvex Optimization and Its Applications
=#

# obs: example 2 is from the book

# Cap 3.2, Pag 25
function jump_03(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(LowerToUpper(model), x)
    @variable(UpperToLower(model), y)

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

    @test objective_value(model) == 3* (3.5*8/15) + (8/15)

    @test value(x) == 3.5*8/15
    @test value(y) == 8/15

end
# change the bound on x to lower level
function jump_04(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(LowerToUpper(model), x)
    @variable(UpperToLower(model), y)

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
function jump_05(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(LowerToUpper(model), x[i=1:2])
    @variable(UpperToLower(model), y)

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
function jump_3SAT(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    n = 7 # maximum literals
    clauses = [[1,2,3],[-1,-4,3],[7,-6,4],[5,6,7]]

    @variable(LowerToUpper(model), x[i=1:n])
    @variable(UpperToLower(model), ya[i=1:n])
    @variable(UpperToLower(model), yb[i=1:n])
    @variable(UpperToLower(model), z)
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

    @test objective_value(model) == -1

    # 3SAT is yese IFF obj = -1

end

# sec 5.1 pag 121 -> product

# sec 5.1 pag 127
function jump_quad_01_a(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(LowerToUpper(model), x)
    @variable(UpperToLower(model), y)

    @objective(Upper(model), Min, x^2 + y)
    @constraint(Upper(model), -x -y <= 0)

    @objective(Lower(model), Min, x)
    @constraint(Lower(model), x >= 0)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) == 0
    @test value(x) == 0
    @test value(y) == 0

end
function jump_quad_01_b(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(LowerToUpper(model), x)
    @variable(UpperToLower(model), y)

    @objective(Upper(model), Min, x^2 + y)
    
    @objective(Lower(model), Min, x)
    @constraint(Lower(model), -x -y <= 0)
    @constraint(Lower(model), x >= 0)

    MOI.empty!(optimizer)
    @test MOI.is_empty(optimizer)

    optimize!(model, optimizer, mode)

    primal_status(model)

    termination_status(model)

    @test objective_value(model) == 0.5^2 - 0.5
    @test value(x) == 0.5
    @test value(y) == -0.5

end
function jump_quad_01_c(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(LowerToUpper(model), x)
    @variable(UpperToLower(model), y)

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
function jump_int_01(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(LowerToUpper(model), x)
    @variable(UpperToLower(model), y)

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
function jump_int_02(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(LowerToUpper(model), x)
    @variable(UpperToLower(model), y)

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
function jump_06(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y)

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
function jump_07(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(UpperToLower(model), x[i=1:2])
    @variable(LowerToUpper(model), y[i=1:3])

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
function jump_08(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y)

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
function jump_09a(optimizer, mode = BilevelJuMP.SOS1Mode)

    # degenerate second level
    # its case that show that the KKT approach is OPTIMISTIC

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y[i=1:2])

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
function jump_09b(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y[i=1:2])

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
function jump_10(optimizer, mode = BilevelJuMP.SOS1Mode)

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

    @variable(UpperToLower(model), b)
    @variable(LowerToUpper(model), x[i=1:n])

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

    # @test value(x) == 0
    # @test value(y) == [1, 0]

end

# pag 21 ex 2.1
function jump_11a(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y)

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
function jump_11b(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y)

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
function jump_12(optimizer, mode = BilevelJuMP.SOS1Mode)

    for a in [0 0.1 0.2]
        model = BilevelModel()

        @variable(UpperToLower(model), x)
        @variable(LowerToUpper(model), y)
        @variable(LowerToUpper(model), z)

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
function jump_13_quad(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y)

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
function jump_14(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(UpperToLower(model), y)
    @variable(LowerToUpper(model), z)

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
function jump_15a_INT(optimizer, mode = BilevelJuMP.SOS1Mode)

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y)
    @variable(LowerToUpper(model), z)

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
function jump_15b_INT(optimizer, mode = BilevelJuMP.SOS1Mode)
    # send y to upper level

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(UpperToLower(model), y)
    @variable(LowerToUpper(model), z)

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
    Princeton Handbook of NLP test problems
=#

