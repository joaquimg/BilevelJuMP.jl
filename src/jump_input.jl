using BilevelJuMP
using Ipopt
using JuMP
using Test

model = BilevelModel(Ipopt.Optimizer, mode = BilevelJuMP.ProductMode(1e-9))

@variable(Upper(model), y, start = 8 / 15)
@variable(Lower(model), x, start = 3.5 * 8 / 15)
@objective(Upper(model), Min, 3x + y)
@constraints(Upper(model), begin
    u1, x <= 5
    u2, y <= 8
    u3, y >= 0
end)

@objective(Lower(model), Min, -x)
@constraint(Lower(model), l1,  x +  y <= 8)
@constraint(Lower(model), l2, 4x +  y >= 8)
@constraint(Lower(model), l3, 2x +  y <= 13)
@constraint(Lower(model), l4, 2x - 7y <= 0)

function _build_single_model(
    model::BilevelModel,
)
    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)
    linkUL = JuMP.index(model.upper_to_lower_link)
    linkLU = JuMP.index(model.lower_to_upper_link)

    return _build_single_model(upper, lower, linkUL, linkLU)
end 

function _build_single_model(
    upper::MOI.ModelLike, 
    lower::MOI.ModelLike, 
    # A dictionary that maps variables in the upper to variables in the lower
    linkUL::Dict{MOI.VariableIndex,MOI.VariableIndex},
    linkLU::Dict{MOI.VariableIndex,MOI.VariableIndex}
)
    # A new model to build
    model = MOI.Utilities.Model{Float64}()
    # Create a copy of the upper model
    upper_to_model_link = MOI.copy_to(model, upper)
    upper_variables = [upper_to_model_link[k] for k in keys(upper_to_lower_link)]
    lower_varibales = [index_map[k] for k in values(linkLU)]
    # upper only variables 
    
    #print(index_map)

    # objective function of the lower level

    # constraint of the lower level
    # variable of the lower level
    # variable of the upper level


   # tp_primal_obj = MOI.get(lower, MOI.ObjectiveFunctionType()) 
   # @assert tp_primal_obj !== nothing 
    lower_primal_obj = MOI.get(lower, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}()) 
    MOI.Utilities.map_indices(lower_primal_obj) do x
        return uppertomodel[linkLU[x]]
    end
    # deepcopy and delete dual obj 
    #     MOI.set(lower, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{MOI.ScalarAffineFunction}}(), MOI.ScalarAffineFunction(MOI.ScalarAffineTerm{Float64}[], 0.0))


    # TODO ... add the lower problem to `model`
    return model#, upper_variables, lower_variables, lower_objective
    #return model, upper_variables#, lower_variables, lower_objective
    #return model, upper_variables, lower_variables#, lower_objective
    #return model, upper_variables, lower_variables, lower_objective
end

# optimize!(model)

new_model = _build_single_model(model)
# new_model, upper_variables = _build_single_model(model)
# new_model, upper_variables, lower_variables = _build_single_model(model)
# new_model, upper_variables, lower_variables, lower_objective = _build_single_model(model)
# A = num_variables(new_model)

Num_Var = MOI.get(new_model, MOI.NumberOfVariables())
Num_Cnt = MOI.get(new_model, MOI.NumberOfConstraints())

println("\n===============")
println("Number of variables: ", Num_Var)
println("Number of constraints: ", Num_Cnt)
println("\n===============")
println("Variables:")
List_Var = MOI.get(new_model, MOI.ListOfVariableIndices())

for x in List_Var
    println(MOI.get(new_model, MOI.VariableName(), x))  
    println(MOI.is_valid(new_model, MOI.ConstraintIndex{MOI.SingleVariable, MOI.Integer}(x.value)))   
end

for ci in MOI.get(new_model, MOI.ListOfConstraintIndices{MOI.SingleVariable, MOI.Integer}())
    MOI.VariableIndex(ci.value)
end
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
