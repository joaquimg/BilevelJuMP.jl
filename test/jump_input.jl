using BilevelJuMP
using JuMP
using Test

@testset "basic example (1)" begin
    model = BilevelModel()

    @variable(Upper(model), y, start = 8 / 15)
    @variable(Upper(model), z, start = 8 / 15)
    @variable(Lower(model), x, start = 3.5 * 8 / 15)
    @objective(Upper(model), Min, 3x + y + z)
    @constraints(Upper(model), begin
        u1, x <= 5
        u2, y <= 8
        u3, y >= 0
        u4, z >=0
    end)

    @objective(Lower(model), Min, -x)
    @constraint(Lower(model), l1,  x +  y <= 8)
    @constraint(Lower(model), l2, 4x +  y >= 8)
    @constraint(Lower(model), l3, 2x +  y <= 13)
    @constraint(Lower(model), l4, 2x - 7y <= 0)

    new_model, lower_variables, lower_objective, lower_constraints = BilevelJuMP._build_single_model(model)


    #Check over number of variables
    @test length(lower_variables) == 1 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (2 + 1) # upper + lower

    println(MOI.get(new_model, MOI.NumberOfConstraints()))
    
    #Check over number of constraints
    @test length(lower_constraints) == (4 + 0) #lower constraint + integer variables
    #@test MOI.get(new_model, MOI.NumberOfConstraints()) == (4 + 4 + 0 + 0) # upper lower integer_uppper integer_lower 

    #Check over the objective function of the lower level
    x = lower_variables[1]
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}([MOI.ScalarAffineTerm(-1.0, x)],0.0)

end

@testset "basic example (2) - integer in lower level " begin
    model = BilevelModel()

    @variable(Upper(model), y, start = 8 / 15)
    @variable(Upper(model), z, start = 8 / 15)
    @variable(Lower(model), x, Int, start = 3.5 * 8 / 15)
    @objective(Upper(model), Min, 3x + y + z)
    @constraints(Upper(model), begin
        u1, x <= 5
        u2, y <= 8
        u3, y >= 0
        u4, z >=0
    end)

    @objective(Lower(model), Min, -x)
    @constraint(Lower(model), l1,  x +  y <= 8)
    @constraint(Lower(model), l2, 4x +  y >= 8)
    @constraint(Lower(model), l3, 2x +  y <= 13)
    @constraint(Lower(model), l4, 2x - 7y <= 0)

    new_model, lower_variables, lower_objective, lower_constraints = BilevelJuMP._build_single_model(model)
    

    #Check over number of variables
    @test length(lower_variables) == 1 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (2 + 1) # upper + lower

    #Check over number of constraints
    @test length(lower_constraints) == (4 + 1) #lower constraint + integer variables
    @test MOI.get(new_model, MOI.NumberOfConstraints()) == (4 + 4 + 0 + 1) # upper lower integer_uppper integer_lower 

    #Check over the objective function of the lower level
    x = lower_variables[1]
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}([MOI.ScalarAffineTerm(-1.0, x)],0.0)

end


@testset "basic example (3) - integer in upper level " begin
    model = BilevelModel()

    @variable(Upper(model), y, start = 8 / 15)
    @variable(Upper(model), z, Int, start = 8 / 15)
    @variable(Lower(model), x, start = 3.5 * 8 / 15)
    @objective(Upper(model), Min, 3x + y + z)
    @constraints(Upper(model), begin
        u1, x <= 5
        u2, y <= 8
        u3, y >= 0
        u4, z >=0
    end)

    @objective(Lower(model), Min, -x)
    @constraint(Lower(model), l1,  x +  y <= 8)
    @constraint(Lower(model), l2, 4x +  y >= 8)
    @constraint(Lower(model), l3, 2x +  y <= 13)
    @constraint(Lower(model), l4, 2x - 7y <= 0)

    new_model, lower_variables, lower_objective, lower_constraints = BilevelJuMP._build_single_model(model)

    #Check over number of variables
    @test length(lower_variables) == 1 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (2 + 1) # upper + lower

    #Check over number of constraints
    @test length(lower_constraints) == (4 + 0) #lower constraint + integer variables
    @test MOI.get(new_model, MOI.NumberOfConstraints()) == (4 + 4 + 1 + 0) # upper lower integer_uppper integer_lower 

    #Check over the objective function of the lower level
    x = lower_variables[1]
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}([MOI.ScalarAffineTerm(-1.0, x)],0.0)
end



#=


println("\n===============")
println("Number of variables: ", Num_Var)
#println("Number of constraints: ", Num_Cnt)
println("\n===============")
println("Variables:")
List_Var = MOI.get(new_model, MOI.ListOfVariableIndices())

MOI.write_to_file(new_model, "/Users/hesamshaelaie/Documents/BilevelJuMP.jl/src/test.mps")
new_jump_model = JuMP.read_from_file("/Users/hesamshaelaie/Documents/BilevelJuMP.jl/src/test.mps")
print(new_jump_model)

for x in List_Var
    println(MOI.get(new_model, MOI.VariableName(), x))  
    println(MOI.is_valid(new_model, MOI.ConstraintIndex{MOI.SingleVariable, MOI.Integer}(x.value)))   
end

#=
for ci in MOI.get(new_model, MOI.ListOfConstraintIndices{MOI.SingleVariable, MOI.Integer}())
    MOI.VariableIndex(ci.value)
end
=#


println("\n===============")
typeof(List_Var[1])
#println("Total number of variables:" , Num_Var)

# println(MOI.get(new_model, MOI.VariableName(), lower_variables[1]))
# println(MOI.get(new_model, MOI.VariableName(), lower_variables[2]))
# println(lower_variables)

#=
for (F, S) in MOI.get(new_model, MOI.ListOfConstraints())
    println(F)
    println(S)
end
=#


#@test num_variables(new_model) == 1
#@test length(upper_variables) == 1
#@test length(lower_variables) == 1
#@test objective_function(new_model) ≈ 3 * lower_variables[1] + upper_variables[1]
#@test lower_objective ≈ -lower_variables[1]


# Automated testing

# @test objective_value(model) ≈ 3 * (3.5 * 8 / 15) + (8 / 15) atol=1e-6
# @test BilevelJuMP.lower_objective_value(model) ≈ -3.5 * 8 / 15 atol=1e-6
# @test objective_value(Lower(model)) ≈ -3.5 * 8 / 15 atol=1e-6
# @test value(x) ≈ 3.5 * 8 / 15 atol=1e-6
# @test value(y) ≈ 8 / 15 atol=1e-6
# @test value(u1) ≈ 3.5 * 8 / 15 atol=1e-6
# @test value(l1) ≈ 4.5 * 8 / 15 atol=1e-6
# @test dual(l1) ≈ [0] atol=1e-6
# @test dual(l3) ≈ [0] atol=1e-6

# TODO: why are these commented out?    #src
# @test dual(l2) #≈ [0] atol=atol       #src
# @test dual(l4) #≈ [0] atol=atol       #src
=#