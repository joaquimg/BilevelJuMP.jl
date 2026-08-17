# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

module TestMIBS

using BilevelJuMP
using Test

# Tests named `test_solver_*` run the MibS binary; the rest only build the files
# that would be handed to it, and so can run anywhere.
const MIBS_AVAILABLE = try
    @eval using MibS_jll
    MibS_jll.is_available()
catch
    false
end

mibs_call() = MibS_jll.mibs

function runtests()
    if !MIBS_AVAILABLE
        @warn "MibS is unavailable on this platform, running only the tests " *
              "that do not need the binary."
    end
    for name in names(@__MODULE__; all = true)
        s = "$(name)"
        if startswith(s, "test_")
            if startswith(s, "test_solver_") && !MIBS_AVAILABLE
                continue
            end
            @testset "$(s)" begin
                getfield(@__MODULE__, name)()
            end
        end
    end
    return
end

function test_build_single_model_structure()
    new_model,
    lower_variables,
    lower_objective,
    lower_constraints,
    lower_sense = BilevelJuMP._build_single_model(_mixed_sense_model())
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
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}(
        [MOI.ScalarAffineTerm(-1.0, lower_variables[1])],
        0.0,
    )
    @test lower_sense == MOI.MIN_SENSE
    return
end

function test_basic_example_4()
    model = BilevelModel()
    I = 7 # maximum literals
    clauses = [[1, 2, 3], [-1, -4, 3], [7, -6, 4], [5, 6, 7]]
    @variable(Upper(model), ya[i=1:I])          #7 variables
    @variable(Upper(model), yb[i=1:I])          #7 variables
    @variable(Upper(model), z)                  #1 variable
    @variable(Lower(model), x[i=1:I])           #7 variables
    @objective(Upper(model), Min, sum(x[i] for i in 1:I) - z)
    @constraint(Upper(model), ca, z <= 1)                       #1 LessThan
    @constraint(Upper(model), cb, z >= 0)                       #1 GreaterThan
    @constraint(Upper(model), c1[i=1:I], ya[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c2[i=1:I], ya[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c3[i=1:I], yb[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c4[i=1:I], yb[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c5[i=1:I], ya[i] + yb[i] == 1)    #7 Equlity
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
    @constraint(Lower(model), b1[i=1:I], x[i] >= 0)         #7  GreaterThan
    @constraint(Lower(model), b2[i=1:I], x[i] <= ya[i])     #7  LessThan
    @constraint(Lower(model), b3[i=1:I], x[i] <= yb[i])     #7  LessThan
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
    @variable(Upper(model), ya[i=1:I])          #7 variables
    @variable(Upper(model), yb[i=1:I])          #7 variables
    @variable(Upper(model), z)                  #1 variable
    @variable(Lower(model), x[i=1:I], Int)           #7 variables
    @objective(Upper(model), Min, sum(x[i] for i in 1:I) - z)
    @constraint(Upper(model), ca, z <= 1)                       #1 LessThan
    @constraint(Upper(model), cb, z >= 0)                       #1 GreaterThan
    @constraint(Upper(model), c1[i=1:I], ya[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c2[i=1:I], ya[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c3[i=1:I], yb[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c4[i=1:I], yb[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c5[i=1:I], ya[i] + yb[i] == 1)    #7 Equlity
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
    @constraint(Lower(model), b1[i=1:I], x[i] >= 0)         #7  GreaterThan
    @constraint(Lower(model), b2[i=1:I], x[i] <= ya[i])     #7  LessThan
    @constraint(Lower(model), b3[i=1:I], x[i] <= yb[i])     #7  LessThan
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
    @variable(Upper(model), ya[i=1:I])                          #7 variables
    @variable(Upper(model), yb[i=1:I], Int)                     #7 variables
    @variable(Upper(model), z, Int)                             #1 variable
    @variable(Lower(model), x[i=1:I])                           #7 variables
    @objective(Upper(model), Min, sum(x[i] for i in 1:I) - z)
    @constraint(Upper(model), ca, z <= 1)                       #1 LessThan
    @constraint(Upper(model), cb, z >= 0)                       #1 GreaterThan
    @constraint(Upper(model), c1[i=1:I], ya[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c2[i=1:I], ya[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c3[i=1:I], yb[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c4[i=1:I], yb[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c5[i=1:I], ya[i] + yb[i] == 1)    #7 Equlity
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
    @constraint(Lower(model), b1[i=1:I], x[i] >= 0)         #7  GreaterThan
    @constraint(Lower(model), b2[i=1:I], x[i] <= ya[i])     #7  LessThan
    @constraint(Lower(model), b3[i=1:I], x[i] <= yb[i])     #7  LessThan
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

function test_solver_example_1()
    model = BilevelModel()
    BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(mibs_call()))
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
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    @test objective_value(model) ≈ 8
    @test value(y) ≈ 8
    @test value(z) ≈ 0
    @test value(x) ≈ 0
    return
end

function test_solver_example_2()
    model = BilevelModel()
    BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(mibs_call()))
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
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    @test objective_value(model) ≈ -53
    @test value(x) ≈ 6
    @test value(y) ≈ 5
    return
end

function test_solver_example_3()
    model = BilevelModel()
    BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(mibs_call()))
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
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    @test objective_value(model) ≈ -22
    @test value(x) ≈ 2
    @test value(y) ≈ 2
    return
end

function test_solver_example_4()
    model = BilevelModel()
    BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(mibs_call()))
    @variable(Upper(model), x, Int)
    @variable(Lower(model), y, Int)
    @objective(Upper(model), Min, -x - 10y)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, -25x + 20y <= 30)
    @constraint(Lower(model), l2, x + 2y <= 10)
    @constraint(Lower(model), l3, 2x - y <= 15)
    @constraint(Lower(model), l4, -2x - 10y <= -15)
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    @test objective_value(model) ≈ -22
    @test value(x) ≈ 2
    @test value(y) ≈ 2
    return
end

# Two upper level variables, which is what makes the per-variable values worth
# asserting: they used to be read out by position within a block.
function test_solver_example_5()
    model = BilevelModel()
    BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(mibs_call()))
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
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    @test objective_value(model) ≈ -19
    @test value(x) ≈ 2
    @test value(z) ≈ 1
    @test value(y) ≈ 2
    return
end

function test_solver_example_6()
    model = BilevelModel()
    BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(mibs_call()))
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
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    @test objective_value(model) ≈ -19
    @test value(x) ≈ 1
    @test value(z) ≈ 2
    @test value(y) ≈ 2
    return
end

function test_solver_infeasible()
    model = BilevelModel()
    BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(mibs_call()))
    @variable(Upper(model), x, Int)
    @variable(Lower(model), y, Int)
    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), u1, x <= 1)
    @constraint(Upper(model), u2, x >= 3)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, y <= 5)
    @constraint(Lower(model), l2, y >= 0)
    optimize!(model)
    @test termination_status(model) == MOI.INFEASIBLE
    @test primal_status(model) == MOI.NO_SOLUTION
    @test occursin("infeasible", raw_status(model))
    return
end

#=
    MibS identifies the lower level rows and columns by their position in the MPS
    file, so the auxiliary file is only correct if those positions agree with the
    order the MPS writer used. Asserting on the file contents is what catches a
    disagreement: the objective value alone does not, because a mislabeled model
    can still happen to have the same optimum.
=#

# Independent re-derivation of the row/column order, by parsing the MPS file that
# was handed to MibS. Deliberately does not reuse the package's parser.
function _reference_order(mps_filename)
    rows, columns = String[], String[]
    section = :none
    for line in eachline(mps_filename)
        isempty(strip(line)) && continue
        if !isspace(first(line))
            keyword = uppercase(first(split(line)))
            section =
                keyword == "ROWS" ? :rows :
                keyword == "COLUMNS" ? :columns : :other
            continue
        end
        fields = split(line)
        if section == :rows && uppercase(fields[1]) != "N"
            push!(rows, fields[2])
        elseif section == :columns && !any(f -> occursin('\'', f), fields)
            fields[1] in columns || push!(columns, fields[1])
        end
    end
    return rows, columns
end

function _write_mibs_files(model)
    dir = mktempdir()
    mps = joinpath(dir, "m.mps")
    aux = joinpath(dir, "m.aux")
    new_model, variables, objective, constraints, sense =
        BilevelJuMP._build_single_model(model, true)
    MOI.write_to_file(new_model, mps)
    BilevelJuMP._write_auxiliary_file(
        new_model,
        variables,
        objective,
        constraints,
        sense,
        mps,
        aux,
    )
    return new_model, variables, constraints, mps, aux
end

# Both `<=` and `>=` rows are needed: the two set types are what get transposed if
# the row order is assumed rather than read back from the file.
function _mixed_sense_model()
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
    return model
end

function test_auxiliary_file_indices_match_the_mps_file()
    new_model, variables, constraints, mps, aux =
        _write_mibs_files(_mixed_sense_model())
    rows, columns = _reference_order(mps)
    lines = readlines(aux)
    lr = sort([parse(Int, split(l)[2]) for l in lines if startswith(l, "LR ")])
    lc = sort([parse(Int, split(l)[2]) for l in lines if startswith(l, "LC ")])
    expected_lr = sort(map(constraints) do ci
        name = MOI.get(new_model, MOI.ConstraintName(), ci)
        return findfirst(isequal(name), rows) - 1
    end)
    expected_lc = sort(map(variables) do vi
        name = MOI.get(new_model, MOI.VariableName(), vi)
        return findfirst(isequal(name), columns) - 1
    end)
    @test lr == expected_lr
    @test lc == expected_lc
    # Pinned so a future change of order fails here, not silently.
    @test lr == [2, 3, 4, 7]
    @test lc == [2]
    # Every declared lower row must be a real row of the file.
    @test all(0 .<= lr .< length(rows))
    @test length(lr) == length(constraints)
    return
end

function test_auxiliary_file_header_counts()
    _, variables, constraints, _, aux = _write_mibs_files(_mixed_sense_model())
    lines = readlines(aux)
    @test lines[1] == "N $(length(variables))"
    @test lines[2] == "M $(length(constraints))"
    @test count(l -> startswith(l, "LC "), lines) == length(variables)
    @test count(l -> startswith(l, "LR "), lines) == length(constraints)
    @test count(l -> startswith(l, "LO "), lines) == length(variables)
    @test last(lines) == "OS 1"
    return
end

function test_maximization_lower_objective_sense()
    model = BilevelModel()
    @variable(Upper(model), y, Int)
    @variable(Lower(model), x, Int)
    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), u1, y >= 0)
    @objective(Lower(model), Max, x)
    @constraint(Lower(model), l1, x + y <= 8)
    _, _, _, _, aux = _write_mibs_files(model)
    @test last(readlines(aux)) == "OS -1"
    return
end

#=
    Models MibS cannot represent must be rejected with an actionable error rather
    than silently solved as a different problem, crashed on, or looped on forever.
=#

function test_unsupported_continuous_variable()
    model = BilevelModel()
    @variable(Upper(model), y)
    @variable(Lower(model), x, Int)
    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), u1, y >= 0)
    @objective(Lower(model), Min, -x)
    @constraint(Lower(model), l1, x + y <= 8)
    @test_throws ErrorException BilevelJuMP._build_single_model(model, true)
    # The check is opt-out, and it is the only thing `check_integrality` gates.
    @test BilevelJuMP._build_single_model(model, false) isa Tuple
    return
end

function test_unsupported_lower_only_variable()
    model = BilevelModel()
    @variable(Upper(model), y, Int)
    @variable(Lower(model), x, Int)
    @variable(LowerOnly(model), w, Int)
    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), u1, y >= 0)
    @objective(Lower(model), Min, -x)
    @constraint(Lower(model), l1, x + y + w <= 8)
    @test_throws ErrorException BilevelJuMP._build_single_model(model, true)
    return
end

function test_unsupported_dual_of_variable()
    model = BilevelModel()
    @variable(Upper(model), y, Int)
    @variable(Lower(model), x, Int)
    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), u1, y >= 0)
    @objective(Lower(model), Min, -x)
    @constraint(Lower(model), l1, x + y <= 8)
    @variable(Upper(model), lambda, DualOf(l1))
    @test_throws ErrorException BilevelJuMP._build_single_model(model, true)
    return
end

function test_unsupported_missing_lower_objective()
    model = BilevelModel()
    @variable(Upper(model), y, Int)
    @variable(Lower(model), x, Int)
    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), u1, y >= 0)
    @constraint(Lower(model), l1, x + y <= 8)
    @test_throws ErrorException BilevelJuMP._build_single_model(model, true)
    return
end

function test_unsupported_quadratic_data()
    model = BilevelModel()
    @variable(Upper(model), y, Int)
    @variable(Lower(model), x, Int)
    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), u1, y >= 0)
    @objective(Lower(model), Min, x^2)
    @constraint(Lower(model), l1, x + y <= 8)
    @test_throws ErrorException BilevelJuMP._build_single_model(model, true)
    model2 = BilevelModel()
    @variable(Upper(model2), b, Int)
    @variable(Lower(model2), a, Int)
    @objective(Upper(model2), Min, a + b)
    @constraint(Upper(model2), q1, a * b <= 4)
    @objective(Lower(model2), Min, -a)
    @constraint(Lower(model2), q2, a + b <= 8)
    @test_throws ErrorException BilevelJuMP._build_single_model(model2, true)
    return
end

function test_unsupported_conic_constraint()
    model = BilevelModel()
    @variable(Upper(model), y, Int)
    @variable(Lower(model), x[1:3], Int)
    @objective(Upper(model), Min, x[1] + y)
    @constraint(Upper(model), u1, y >= 0)
    @objective(Lower(model), Min, -x[1])
    @constraint(Lower(model), l1, x in SecondOrderCone())
    @test_throws ErrorException BilevelJuMP._build_single_model(model, true)
    return
end

#=
    Solving through `MibSMode`, where results are queried with the ordinary JuMP
    functions rather than from a returned NamedTuple.
=#

function _mode_model(; sense = MOI.MIN_SENSE)
    model = BilevelModel()
    BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(mibs_call()))
    @variable(Upper(model), x, Int)
    @variable(Lower(model), y, Int)
    if sense == MOI.MAX_SENSE
        @objective(Upper(model), Max, 3x + 7y)
    else
        @objective(Upper(model), Min, -3x - 7y)
    end
    @constraints(Upper(model), begin
        u1, -3x + 2y <= 12
        u2, x + 2y <= 20
        u3, x <= 10
    end)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, 2x - y <= 7)
    @constraint(Lower(model), l2, -2x + 4y <= 16)
    @constraint(Lower(model), l3, y <= 5)
    return model, x, y, l1
end

function test_solver_mode_solve_and_query()
    model, x, y, l1 = _mode_model()
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    @test primal_status(model) == MOI.FEASIBLE_POINT
    @test solver_name(model) == "MibS"
    @test occursin("Optimal", raw_status(model))
    @test value(x) ≈ 6
    @test value(y) ≈ 5
    @test objective_value(model) ≈ -53
    @test objective_value(Upper(model)) ≈ -53
    @test objective_value(Lower(model)) ≈ 5
    # No solver to ask, so the constraint function is evaluated instead.
    @test value(l1) ≈ 2 * 6 - 5
    @test BilevelJuMP.build_time(model) >= 0
    @test solve_time(model) >= 0
    return
end

# The MPS file carries a negated objective when the upper level is a
# maximization, so the value has to be evaluated rather than read from MibS.
function test_solver_mode_maximization_upper_objective()
    model, x, y, _ = _mode_model(; sense = MOI.MAX_SENSE)
    optimize!(model)
    @test value(x) ≈ 6
    @test value(y) ≈ 5
    @test objective_value(model) ≈ 53
    return
end

function test_solver_mode_binary_variable()
    model = BilevelModel()
    BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode(mibs_call()))
    @variable(Upper(model), x, Int)
    @variable(Upper(model), z, Bin)
    @variable(Lower(model), y, Int)
    @objective(Upper(model), Min, 2x - 4y + 10z)
    @constraints(Upper(model), begin
        u1, -3x + 2y + 5z <= 12
        u2, x + 2y <= 20
        u3, x <= 10
    end)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, 2x - y + 3z <= 7)
    @constraint(Lower(model), l2, -2x + 4y <= 16)
    @constraint(Lower(model), l3, y <= 5)
    optimize!(model)
    @test objective_value(model) ≈ -8
    @test value(x) ≈ 6
    @test value(z) ≈ 0
    @test value(y) ≈ 5
    return
end

function test_solver_mode_queries_that_need_an_optimizer()
    model, x, _, l1 = _mode_model()
    optimize!(model)
    @test_throws ErrorException dual(l1)
    @test_throws ErrorException dual_status(Upper(model))
    @test_throws ErrorException dual_status(Lower(model))
    @test_throws ErrorException objective_bound(model)
    @test_throws ErrorException relative_gap(model)
    @test_throws ErrorException set_silent(model)
    @test_throws ErrorException set_time_limit_sec(model, 10.0)
    return
end

function test_mode_requires_an_executable()
    model = BilevelModel()
    BilevelJuMP.set_mode(model, BilevelJuMP.MibSMode())
    @variable(Upper(model), x, Int)
    @variable(Lower(model), y, Int)
    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), u1, x <= 5)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, x + y <= 8)
    @test_throws ErrorException optimize!(model)
    return
end

function test_solver_mode_results_before_optimize()
    model, x, _, _ = _mode_model()
    @test_throws ErrorException termination_status(model)
    @test_throws ErrorException objective_value(model)
    @test_throws ErrorException value(x)
    return
end

function test_solver_mode_rejects_an_attached_optimizer()
    model, _, _, _ = _mode_model()
    set_optimizer(
        model,
        () -> MOI.Utilities.MockOptimizer(
            MOI.Utilities.UniversalFallback(MOI.Utilities.Model{Float64}()),
        ),
    )
    @test_throws ErrorException optimize!(model)
    return
end

function test_no_mode_selected()
    model = BilevelModel()
    @variable(Upper(model), x)
    @variable(Lower(model), y)
    @objective(Upper(model), Min, x + y)
    @constraint(Upper(model), u1, x <= 5)
    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1, x + y <= 8)
    @test_throws ErrorException optimize!(model)
    return
end

end  # module TestMIBS

TestMIBS.runtests()
