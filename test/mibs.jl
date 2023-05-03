module TestJuMPInput

using BilevelJuMP
using Test
using MibS_jll

function runtests()
    for name in names(@__MODULE__; all = true)
        # if startswith("$(name)", "test_")
        if startswith("$(name)", "test_")
            @testset "$(name)" begin
                getfield(@__MODULE__, name)()
            end
        end
    end
end

function test_basic_example_1()
    # using BilevelJuMP, MibS_jll
    model = BilevelModel()
    @variable(Upper(model), y, Int)
    @variable(Upper(model), z, Int)
    @variable(Lower(model), x, Int)
    @objective(Upper(model), Min, 3x + y + z)
    @constraints(Upper(model), begin
        u1, x <= 5
        u2, y <= 8
        u3, y >= 0
        u4, z >= 0
    end)
    @objective(Lower(model), Min, -x)
    @constraint(Lower(model), l1, x + y <= 8)
    @constraint(Lower(model), l2, 4x + y >= 8)
    @constraint(Lower(model), l3, 2x + y <= 13)
    @constraint(Lower(model), l4, 2x - 7y <= 0)
    new_model,
    lower_variables,
    lower_objective,
    lower_constraints,
    lower_sense = BilevelJuMP._build_single_model(model)
    @test length(lower_variables) == 1
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (2 + 1)
    @test length(lower_constraints) == (4 + 0)
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.LessThan{Float64},
        }(),
    ) == (2 + 3)
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.GreaterThan{Float64},
        }(),
    ) == (2 + 1)
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{MOI.VariableIndex,MOI.Integer}(),
    ) == 3
    x = lower_variables[1]
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}(
        [MOI.ScalarAffineTerm(-1.0, x)],
        0.0,
    )
    @test lower_sense == MOI.MIN_SENSE
    solution = BilevelJuMP.solve_with_MibS(
        model,
        MibS_jll.mibs;
        verbose_results = true,
        verbose_files = true,
        keep_files = true,
    )
    @test solution.status == true
    @test solution.objective ≈ 8
    @test solution.nonzero_upper == Dict(0 => 8)
    @test solution.nonzero_lower == Dict{Int,Float64}()
    return
end

function test_basic_example_4()
    model = BilevelModel()
    I = 7 # maximum literals
    clauses = [[1, 2, 3], [-1, -4, 3], [7, -6, 4], [5, 6, 7]]
    @variable(Upper(model), ya[i = 1:I])          #7 variables
    @variable(Upper(model), yb[i = 1:I])          #7 variables
    @variable(Upper(model), z)                  #1 variable
    @variable(Lower(model), x[i = 1:I])           #7 variables
    @objective(Upper(model), Min, sum(x[i] for i in 1:I) - z)
    @constraint(Upper(model), ca, z <= 1)                       #1 LessThan
    @constraint(Upper(model), cb, z >= 0)                       #1 GreaterThan
    @constraint(Upper(model), c1[i = 1:I], ya[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c2[i = 1:I], ya[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c3[i = 1:I], yb[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c4[i = 1:I], yb[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c5[i = 1:I], ya[i] + yb[i] == 1)    #7 Equlity
    @constraint(
        Upper(model),
        cc[k in eachindex(clauses)],
        sum(i > 0 ? ya[i] : yb[-i] for i in clauses[k]) >= z
    ) #4 GreaterThan
    #---------------------------------------------------------
    #   19  GreaterThan
    #   15  LessThan
    #   7   Equlity
    #   0   Integer
    #   31  Total
    #---------------------------------------------------------
    @objective(Lower(model), Max, sum(x[i] for i in 1:I))
    @constraint(Lower(model), b1[i = 1:I], x[i] >= 0)         #7  GreaterThan
    @constraint(Lower(model), b2[i = 1:I], x[i] <= ya[i])     #7  LessThan
    @constraint(Lower(model), b3[i = 1:I], x[i] <= yb[i])     #7  LessThan
    #---------------------------------------------------------
    #   7   GreaterThan
    #   14  LessThan
    #   0   Equlity
    #   0   Integer
    #   21  Total
    #---------------------------------------------------------
    new_model,
    lower_variables,
    lower_objective,
    lower_constraints,
    lower_sense = BilevelJuMP._build_single_model(model)
    @test length(lower_variables) == 7
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (15 + 7)
    @test length(lower_constraints) == (21 + 0)
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.LessThan{Float64},
        }(),
    ) == (15 + 14)
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.GreaterThan{Float64},
        }(),
    ) == (19 + 7)
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.EqualTo{Float64},
        }(),
    ) == (7 + 0)
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{MOI.VariableIndex,MOI.Integer}(),
    ) == (0 + 0)
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}(
        [MOI.ScalarAffineTerm(1.0, x) for x in lower_variables],
        0.0,
    )
    @test lower_sense == MOI.MAX_SENSE
    return
end

function test_basic_example_5_integer_in_lower_level()
    model = BilevelModel()
    I = 7 # maximum literals
    clauses = [[1, 2, 3], [-1, -4, 3], [7, -6, 4], [5, 6, 7]]
    @variable(Upper(model), ya[i = 1:I])          #7 variables
    @variable(Upper(model), yb[i = 1:I])          #7 variables
    @variable(Upper(model), z)                  #1 variable
    @variable(Lower(model), x[i = 1:I], Int)           #7 variables
    @objective(Upper(model), Min, sum(x[i] for i in 1:I) - z)
    @constraint(Upper(model), ca, z <= 1)                       #1 LessThan
    @constraint(Upper(model), cb, z >= 0)                       #1 GreaterThan
    @constraint(Upper(model), c1[i = 1:I], ya[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c2[i = 1:I], ya[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c3[i = 1:I], yb[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c4[i = 1:I], yb[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c5[i = 1:I], ya[i] + yb[i] == 1)    #7 Equlity
    @constraint(
        Upper(model),
        cc[k in eachindex(clauses)],
        sum(i > 0 ? ya[i] : yb[-i] for i in clauses[k]) >= z
    ) #4 GreaterThan
    #---------------------------------------------------------
    #   19  GreaterThan
    #   15  LessThan
    #   7   Equlity
    #   0   Integer
    #   31  Total
    #---------------------------------------------------------
    @objective(Lower(model), Max, sum(x[i] for i in 1:I))
    @constraint(Lower(model), b1[i = 1:I], x[i] >= 0)         #7  GreaterThan
    @constraint(Lower(model), b2[i = 1:I], x[i] <= ya[i])     #7  LessThan
    @constraint(Lower(model), b3[i = 1:I], x[i] <= yb[i])     #7  LessThan
    #---------------------------------------------------------
    #   7   GreaterThan
    #   14  LessThan
    #   0   Equlity
    #   7   Integer
    #   21  Total
    #---------------------------------------------------------
    new_model,
    lower_variables,
    lower_objective,
    lower_constraints,
    lower_sense = BilevelJuMP._build_single_model(model)
    @test length(lower_variables) == 7 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (15 + 7) # upper + lower
    @test length(lower_constraints) == 21
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.LessThan{Float64},
        }(),
    ) == (15 + 14) # GreaterThan
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.GreaterThan{Float64},
        }(),
    ) == (19 + 7) # LessThan
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.EqualTo{Float64},
        }(),
    ) == (7 + 0) # EqualTo
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{MOI.VariableIndex,MOI.Integer}(),
    ) == (0 + 7) # number of constraints for integer represantation of the variables
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}(
        [MOI.ScalarAffineTerm(1.0, x) for x in lower_variables],
        0.0,
    )
    @test lower_sense == MOI.MAX_SENSE
    return
end

function test_basic_example_6_integer_in_lower_level()
    model = BilevelModel()
    I = 7 # maximum literals
    clauses = [[1, 2, 3], [-1, -4, 3], [7, -6, 4], [5, 6, 7]]
    @variable(Upper(model), ya[i = 1:I])                          #7 variables
    @variable(Upper(model), yb[i = 1:I], Int)                     #7 variables
    @variable(Upper(model), z, Int)                             #1 variable
    @variable(Lower(model), x[i = 1:I])                           #7 variables
    @objective(Upper(model), Min, sum(x[i] for i in 1:I) - z)
    @constraint(Upper(model), ca, z <= 1)                       #1 LessThan
    @constraint(Upper(model), cb, z >= 0)                       #1 GreaterThan
    @constraint(Upper(model), c1[i = 1:I], ya[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c2[i = 1:I], ya[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c3[i = 1:I], yb[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c4[i = 1:I], yb[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c5[i = 1:I], ya[i] + yb[i] == 1)    #7 Equlity
    @constraint(
        Upper(model),
        cc[k in eachindex(clauses)],
        sum(i > 0 ? ya[i] : yb[-i] for i in clauses[k]) >= z
    ) #4 GreaterThan
    #---------------------------------------------------------
    #   19  GreaterThan
    #   15  LessThan
    #   7   Equlity
    #   8   Integer
    #   31  Total
    #---------------------------------------------------------
    @objective(Lower(model), Max, sum(x[i] for i in 1:I))
    @constraint(Lower(model), b1[i = 1:I], x[i] >= 0)         #7  GreaterThan
    @constraint(Lower(model), b2[i = 1:I], x[i] <= ya[i])     #7  LessThan
    @constraint(Lower(model), b3[i = 1:I], x[i] <= yb[i])     #7  LessThan
    #---------------------------------------------------------
    #   7   GreaterThan
    #   14  LessThan
    #   0   Equlity
    #   0   Integer
    #   21  Total
    #---------------------------------------------------------
    new_model,
    lower_variables,
    lower_objective,
    lower_constraints,
    lower_sense = BilevelJuMP._build_single_model(model)
    @test length(lower_variables) == 7 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (15 + 7) # upper + lower
    @test length(lower_constraints) == (21 + 0) #lower constraint + number of integer variables
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.LessThan{Float64},
        }(),
    ) == (15 + 14) # GreaterThan
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.GreaterThan{Float64},
        }(),
    ) == (19 + 7) # LessThan
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{
            MOI.ScalarAffineFunction{Float64},
            MOI.EqualTo{Float64},
        }(),
    ) == (7 + 0) # EqualTo
    @test MOI.get(
        new_model,
        MOI.NumberOfConstraints{MOI.VariableIndex,MOI.Integer}(),
    ) == (8 + 0) # number of constraints for integer represantation of the variables
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}(
        [MOI.ScalarAffineTerm(1.0, x) for x in lower_variables],
        0.0,
    )
    @test lower_sense == MOI.MAX_SENSE
    return
end

function test_Writing_MibS_input_v1()
    model = BilevelModel()
    @variable(Upper(model), y, Int)
    @variable(Upper(model), z, Int)
    @variable(Lower(model), x, Int)
    @objective(Upper(model), Min, 3x + y + z)
    @constraints(Upper(model), begin
        u1, x <= 5
        u2, y <= 8
        u3, y >= 0
        u4, z >= 0
    end)
    @objective(Lower(model), Min, -x)
    @constraint(Lower(model), l1, x + y <= 8)
    @constraint(Lower(model), l2, 4x + y >= 8)
    @constraint(Lower(model), l3, 2x + y <= 13)
    @constraint(Lower(model), l4, 2x - 7y <= 0)
    solution =
        BilevelJuMP.solve_with_MibS(model, MibS_jll.mibs; verbose_files = true)
    @test solution.status == true
    @test solution.objective ≈ 8
    @test solution.nonzero_upper == Dict(0 => 8)
    @test solution.nonzero_lower == Dict{Int,Float64}()
    return
end

function test_Writing_MibS_input_v2()
    model = BilevelModel()
    @variable(Upper(model), x, Int)
    @variable(Lower(model), y, Int)
    @objective(Upper(model), Min, -3x - 7y)
    @constraints(Upper(model), begin
        u1, -3x + 2y <= 12
        u2, x + 2y <= 20
        u3, x <= 10
    end)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, 2x - y <= 7)
    @constraint(Lower(model), l2, -2x + 4y <= 16)
    @constraint(Lower(model), l3, y <= 5)
    solution = BilevelJuMP.solve_with_MibS(model, MibS_jll.mibs)
    @test solution.status == true
    @test solution.objective ≈ -53
    @test solution.nonzero_upper == Dict(0 => 6.0)
    @test solution.nonzero_lower == Dict(0 => 5.0)
    @test solution.all_upper["x"] == 6.0
    @test solution.all_lower["y"] == 5.0
    return
end

function test_Writing_MibS_input_v3()
    model = BilevelModel()
    @variable(Upper(model), x, Int)
    @variable(Lower(model), y, Int)
    @objective(Upper(model), Min, -x - 10y)
    @constraint(Upper(model), u1, x <= 10)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, -25x + 20y <= 30)
    @constraint(Lower(model), l2, x + 2y <= 10)
    @constraint(Lower(model), l3, 2x - y <= 15)
    @constraint(Lower(model), l4, -2x - 10y <= -15)
    @constraint(Lower(model), l5, y <= 5)
    solution = BilevelJuMP.solve_with_MibS(model, MibS_jll.mibs)
    @test solution.status == true
    @test solution.objective ≈ -22
    @test solution.nonzero_upper == Dict(0 => 2.0)
    @test solution.nonzero_lower == Dict(0 => 2.0)
    return
end

function test_Writing_MibS_input_v4()
    model = BilevelModel()
    @variable(Upper(model), x, Int)
    @variable(Lower(model), y, Int)
    @objective(Upper(model), Min, -x - 10y)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, -25x + 20y <= 30)
    @constraint(Lower(model), l2, x + 2y <= 10)
    @constraint(Lower(model), l3, 2x - y <= 15)
    @constraint(Lower(model), l4, -2x - 10y <= -15)
    solution = BilevelJuMP.solve_with_MibS(model, MibS_jll.mibs)
    @test solution.status == true
    @test solution.objective ≈ -22
    @test solution.nonzero_upper == Dict(0 => 2.0)
    @test solution.nonzero_lower == Dict(0 => 2.0)
    return
end

function test_Writing_MibS_input_v5()
    model = BilevelModel()
    @variable(Upper(model), x, Int)
    @variable(Upper(model), z, Int)
    @variable(Lower(model), y, Int)
    @objective(Upper(model), Min, -z - 10y + x)
    @constraint(Upper(model), u1, 25z + 2x <= 30)
    @constraint(Upper(model), u2, x <= 6)
    @constraint(Upper(model), u3, x >= 2)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, -25x + 20y <= 30)
    @constraint(Lower(model), l2, x + 2y <= 10)
    @constraint(Lower(model), l3, 2x - y <= 15)
    @constraint(Lower(model), l4, -2x - 10y <= -15)
    solution = BilevelJuMP.solve_with_MibS(model, MibS_jll.mibs)
    @test solution.status == true
    @test solution.objective ≈ -19
    #@test solution.nonzero_upper == Dict(0 => 1.0)
    #@test solution.nonzero_upper == Dict(0 => 2.0)
    #@test solution.nonzero_lower == Dict(0 => 2.0)
    @test solution.all_upper["x"] == 2
    @test solution.all_upper["z"] == 1
    @test solution.all_lower["y"] == 2
    return
end

function test_Writing_MibS_input_v6()
    model = BilevelModel()
    @variable(Upper(model), x, Int)
    @variable(Upper(model), z, Int)
    @variable(Lower(model), y, Int)
    @objective(Upper(model), Min, -x - 10y + z)
    @constraint(Upper(model), u1, 25x + 2z <= 30)
    @constraint(Upper(model), u2, z <= 6)
    @constraint(Upper(model), u3, z >= 2)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, -25z + 20y <= 30)
    @constraint(Lower(model), l2, z + 2y <= 10)
    @constraint(Lower(model), l3, 2z - y <= 15)
    @constraint(Lower(model), l4, -2z - 10y <= -15)
    solution = BilevelJuMP.solve_with_MibS(model, MibS_jll.mibs)
    @test solution.status == true
    @test solution.objective ≈ -19
    #@test solution.nonzero_upper == Dict(0 => 2.0)
    #@test solution.nonzero_upper == Dict(0 => 1.0)
    #@test solution.nonzero_lower == Dict(0 => 2.0)
    @test solution.all_upper["x"] == 1
    @test solution.all_upper["z"] == 2
    @test solution.all_lower["y"] == 2
    return
end

end  # module

TestJuMPInput.runtests()
