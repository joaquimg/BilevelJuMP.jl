using BilevelJuMP
using Test, Gurobi, MathOptInterface, JuMP, Dualization
using MathOptFormat

const MOI  = MathOptInterface
const MOIU = MathOptInterface.Utilities
const MOIB = MathOptInterface.Bridges
const MOIT = MathOptInterface.Test

# original problem
# min -4x -3y
# s.t.
#      2x + y <= 4
#       x +2y <= 4
#       x     >= 0
#           y >= 0
#
# sol x = y = 4/3
# obj = 28/3

# bilevel modifications
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

@testset "Simple LP" begin
    optimizer = Gurobi.Optimizer()
    bridged = MOIB.full_bridge_optimizer(optimizer, Float64)
    MOI.empty!(bridged)
    @test MOI.is_empty(bridged)

    # add 10 variables - only diagonal is relevant
    X = MOI.add_variables(bridged, 2)

    c1 = MOIU.normalize_and_add_constraint(bridged, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(2.0, X[1]),
            MOI.ScalarAffineTerm(1.0, X[2])
        ], 0.0), MOI.EqualTo(4.0))

    c2 = MOIU.normalize_and_add_constraint(bridged, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(1.0, X[1]),
            MOI.ScalarAffineTerm(2.0, X[2])
        ], 0.0), MOI.EqualTo(4.0))

    b1 = MOIU.normalize_and_add_constraint(bridged, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(1.0, X[1])
        ], 0.0), MOI.GreaterThan(0.0))

    b2 = MOIU.normalize_and_add_constraint(bridged, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(1.0, X[2])
        ], 0.0), MOI.GreaterThan(0.0))

    MOI.set(bridged, 
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), 
        MOI.ScalarAffineFunction(MOI.ScalarAffineTerm.([-4.0, -3.0], [X[1], X[2]]), 0.0)
        )
    MOI.set(bridged, MOI.ObjectiveSense(), MOI.MIN_SENSE)
    MOI.optimize!(bridged)

    obj = MOI.get(bridged, MOI.ObjectiveValue())

    @test obj ≈ -9.33333 atol = 1e-2

    Xr = MOI.get(bridged, MOI.VariablePrimal(), X)

    @test Xr ≈ [1.3333, 1.3333] atol = 1e-2

end

@testset "Simple BLP" begin
    optimizer = Gurobi.Optimizer()
    bridged = MOIB.full_bridge_optimizer(optimizer, Float64)
    MOI.empty!(bridged)
    @test MOI.is_empty(bridged)

    X = MOI.add_variables(bridged, 2)

    MOI.set(bridged, MOI.VariableName(), X[1], "x_u")
    MOI.set(bridged, MOI.VariableName(), X[2], "y_u")

    MOI.set(bridged, 
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), 
        MOI.ScalarAffineFunction(
            MOI.ScalarAffineTerm.([-4.0, -3.0], [X[1], X[2]]), 0.0)
        )
    MOI.set(bridged, MOI.ObjectiveSense(), MOI.MIN_SENSE)

    optimizer2 = Gurobi.Optimizer()
    bridged2 = MOIB.full_bridge_optimizer(optimizer2, Float64)
    MOI.empty!(bridged2)
    @test MOI.is_empty(bridged2)

    X2 = MOI.add_variables(bridged2, 2)
    MOI.set(bridged2, MOI.VariableName(), X2[1], "x_l")
    MOI.set(bridged2, MOI.VariableName(), X2[2], "y_l")


    c1 = MOIU.normalize_and_add_constraint(bridged2, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(2.0, X2[1]),
            MOI.ScalarAffineTerm(1.0, X2[2])
        ], 0.0), MOI.LessThan(4.0))
    MOI.set(bridged2, MOI.ConstraintName(), c1, "l_c1")

    c2 = MOIU.normalize_and_add_constraint(bridged2, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(1.0, X2[1]),
            MOI.ScalarAffineTerm(2.0, X2[2])
        ], 0.0), MOI.LessThan(4.0))
    MOI.set(bridged2, MOI.ConstraintName(), c2, "l_c2")

    b1 = MOIU.normalize_and_add_constraint(bridged2, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(1.0, X2[1])
        ], 0.0), MOI.GreaterThan(0.0))
    MOI.set(bridged2, MOI.ConstraintName(), b1, "l_b1")


    b2 = MOIU.normalize_and_add_constraint(bridged2, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(1.0, X2[2])
        ], 0.0), MOI.GreaterThan(0.0))
    MOI.set(bridged2, MOI.ConstraintName(), b2, "l_b12")

    MOI.set(bridged2, 
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), 
        MOI.ScalarAffineFunction(
            MOI.ScalarAffineTerm.([1.0], [X2[2]]), 0.0)
        )
    MOI.set(bridged2, MOI.ObjectiveSense(), MOI.MIN_SENSE)


    links = Dict(X[1] => X2[1], X[2] => X2[2])

    parametric = [X2[1]] 

    blp, _, _ = BilevelJuMP.build_bilivel(bridged, bridged2, links, parametric)


    # MOI.optimize!(bridged)

    # obj = MOI.get(bridged, MOI.ObjectiveValue())

    # @test obj ≈ -9.33333 atol = 1e-2

    # Xr = MOI.get(bridged, MOI.VariablePrimal(), X)

    # @test Xr ≈ [1.3333, 1.3333] atol = 1e-2

    # lp_model = MathOptFormat.MOF.Model()
    # MOI.copy_to(lp_model, blp)
    # MOI.write_to_file(lp_model, "my_model.LP")
    # @show pwd()

    optimizer = Gurobi.Optimizer()
    bridged3 = MOIB.full_bridge_optimizer(optimizer, Float64)
    MOI.empty!(bridged3)
    @test MOI.is_empty(bridged3)
    MOI.copy_to(bridged3, blp)

    MOI.optimize!(bridged3)

end

@testset "Simple BLP2" begin
    optimizer = Gurobi.Optimizer()
    bridged = MOIB.full_bridge_optimizer(optimizer, Float64)
    MOI.empty!(bridged)
    @test MOI.is_empty(bridged)

    x = MOI.add_variable(bridged)
    y = MOI.add_variable(bridged)

    MOI.set(bridged, MOI.VariableName(), x, "x_u")
    MOI.set(bridged, MOI.VariableName(), y, "y_u")

    MOI.set(bridged, 
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), 
        MOI.ScalarAffineFunction(
            MOI.ScalarAffineTerm.([1.0, 3.0], [x, y]), 0.0)
        )
    MOI.set(bridged, MOI.ObjectiveSense(), MOI.MIN_SENSE)

    optimizer2 = Gurobi.Optimizer()
    bridged2 = MOIB.full_bridge_optimizer(optimizer2, Float64)
    MOI.empty!(bridged2)
    @test MOI.is_empty(bridged2)

    x2 = MOI.add_variable(bridged2)
    y2 = MOI.add_variable(bridged2)

    MOI.set(bridged2, MOI.VariableName(), x2, "x_l")
    MOI.set(bridged2, MOI.VariableName(), y2, "y_l")


    c1 = MOIU.normalize_and_add_constraint(bridged2, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(1.0, x2),
            MOI.ScalarAffineTerm(1.0, y2)
        ], 0.0), MOI.LessThan(8.0))
    MOI.set(bridged2, MOI.ConstraintName(), c1, "l_c1")

    c2 = MOIU.normalize_and_add_constraint(bridged2, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(1.0, x2),
            MOI.ScalarAffineTerm(4.0, y2)
        ], 0.0), MOI.GreaterThan(8.0))
    MOI.set(bridged2, MOI.ConstraintName(), c2, "l_c2")

    c3 = MOIU.normalize_and_add_constraint(bridged2, 
    MOI.ScalarAffineFunction([
        MOI.ScalarAffineTerm(1.0, x2),
        MOI.ScalarAffineTerm(2.0, y2)
    ], 0.0), MOI.LessThan(13.0))
    MOI.set(bridged2, MOI.ConstraintName(), c3, "l_c3")

    b1 = MOIU.normalize_and_add_constraint(bridged2, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(1.0, x2)
        ], 0.0), MOI.GreaterThan(1.0))
    MOI.set(bridged2, MOI.ConstraintName(), b1, "l_b1")


    b2 = MOIU.normalize_and_add_constraint(bridged2, 
        MOI.ScalarAffineFunction([
            MOI.ScalarAffineTerm(1.0, x2)
        ], 0.0), MOI.LessThan(6.0))
    MOI.set(bridged2, MOI.ConstraintName(), b2, "l_b12")

    MOI.set(bridged2, 
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), 
        MOI.ScalarAffineFunction(
            MOI.ScalarAffineTerm.([-1.0], [y2]), 0.0)
        )
    MOI.set(bridged2, MOI.ObjectiveSense(), MOI.MIN_SENSE)


    links = Dict(x => x2, y => y2)

    parametric = [x2] 

    blp, _, _ = BilevelJuMP.build_bilivel(bridged, bridged2, links, parametric)


    # MOI.optimize!(bridged)

    # obj = MOI.get(bridged, MOI.ObjectiveValue())

    # @test obj ≈ -9.33333 atol = 1e-2

    # Xr = MOI.get(bridged, MOI.VariablePrimal(), X)

    # @test Xr ≈ [1.3333, 1.3333] atol = 1e-2

    # lp_model = MathOptFormat.MOF.Model()
    # MOI.copy_to(lp_model, blp)
    # MOI.write_to_file(lp_model, "my_model.LP")
    # @show pwd()

    optimizer = Gurobi.Optimizer()
    bridged3 = MOIB.full_bridge_optimizer(optimizer, Float64)
    MOI.empty!(bridged3)
    @test MOI.is_empty(bridged3)
    MOI.copy_to(bridged3, blp)

    MOI.optimize!(bridged3)

end

@testset "Simple BLP JuMP" begin

    model = BilevelModel()

    @variable(UpperToLower(model), x)
    @variable(LowerToUpper(model), y)

    @objective(Upper(model), Min, -4x -3y)

    @objective(Lower(model), Min, y)

    @constraint(Lower(model), 2x+y <= 4)
    @constraint(Lower(model), x+2y <= 4)
    @constraint(Lower(model), x >= 0)
    @constraint(Lower(model), y >= 0)

    optimize!(model, Gurobi.Optimizer())

    primal_status(model)

    termination_status(model)

    @test objective_value(model) == -8

    @test value(x) ==  2
    @test value(y) ==  0

end

@testset "Simple BLP2 JuMP" begin

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

    optimize!(model, Gurobi.Optimizer())

    primal_status(model)

    termination_status(model)

    @test objective_value(model) == 12

    @test value(x) == 6
    @test value(y) == 2

end

# alternative sintax
# @constraint(model, lower, [y] \in ArgMin([x]))
