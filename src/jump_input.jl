using MibS_jll

function _build_single_model(
    model::BilevelModel,
)
    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)
    
    linkLU = Dict(
        # lower to upper
        JuMP.index(v) => JuMP.index(k) for (k,v) in model.link
        # model.link means all link from upper to lower
    )
    linkLOnly = JuMP.index(model.lower_to_upper_link)
    return _build_single_model(upper, lower, linkLU, linkLOnly)
end 


function _build_single_model(
    upper::MOI.ModelLike, 
    lower::MOI.ModelLike, 

    # A dictionary that maps variables in the upper to variables in the lower
    # upper_to_lower_link::Dict{MOI.VariableIndex,MOI.VariableIndex},
    lower_to_upper_link::Dict{MOI.VariableIndex, MOI.VariableIndex},
    lower_only::Dict{MOI.VariableIndex, MOI.VariableIndex}
)
    # A new model to build
    #model = MOI.Utilities.Model{Float64}()
    model = MOI.FileFormats.MPS.Model()
    
    # Create a copy of the upper model
    upper_to_model_link = MOI.copy_to(model, upper)

    #upper_variables = [upper_to_model_link[k] for k in keys(upper_to_lower_link)]
    lower_variables = [upper_to_model_link[k] for k in values(lower_only)]

    
    lower_constraints = Any[]
    for (F, S) in MOI.get(lower, MOI.ListOfConstraints())
        for ci in MOI.get(lower, MOI.ListOfConstraintIndices{F,S}())
            
            lower_f = MOI.get(lower, MOI.ConstraintFunction(), ci)
            lower_s = MOI.get(lower, MOI.ConstraintSet(), ci)
            #@show lower_f, lower_s

            lower_f = MOI.Utilities.map_indices(lower_f) do x
               return upper_to_model_link[lower_to_upper_link[x]]
            end

            new_ci = MOI.add_constraint(model, lower_f, lower_s)
            push!(lower_constraints, new_ci)
        end
    end

    
    lower_primal_obj = MOI.get(lower, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}()) 
    lower_objective = MOI.Utilities.map_indices(lower_primal_obj) do x
        return upper_to_model_link[lower_to_upper_link[x]]
    end

    return model, lower_variables, lower_objective, lower_constraints
end



"""
    index_to_row_link(model::MOI.FileFormats.MPS.Model)

Returns a dictionary that maps the affine constraint indices of `model` to their
row in the MPS file.
"""
function index_to_row_link(model::MOI.FileFormats.MPS.Model)
    i = 0
    dict = Dict{MOI.ConstraintIndex,Int}()
    for (S, _) in MOI.FileFormats.MPS.SET_TYPES
        for ci in MOI.get(
            model,
            MOI.ListOfConstraintIndices{MOI.ScalarAffineFunction{Float64},S}(),
        )  
            dict[ci] = i
            i += 1
        end
    end
    return dict
end

"""
    index_to_column_link(model::MOI.FileFormats.MPS.Model)

Returns a dictionary that maps the variable indices of `model` to their column 
in the MPS file.
"""
function index_to_column_link(model::MOI.FileFormats.MPS.Model)
    variables = MOI.get(model, MOI.ListOfVariableIndices())
    return Dict{MOI.VariableIndex,Int}(
        x => i - 1 for (i, x) in MOI.enumerate(variables)
    )
end



#=
function write_AUX(


    



)
=#

function solve_MibS(
    model::BilevelModel,
)
# Single MIP
# write the MPS
# write the AUX

MOI.write_to_file(new_model, "model.mps")


# call MibS

MibS_jll.mibs() do exe
    run(`$(exe) -Alps_instance model.mps -MibS_auxiliaryInfoFile model.aux`)
end

# return the solution

end

1+1

