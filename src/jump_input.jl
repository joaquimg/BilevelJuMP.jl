using BilevelJuMP
using JuMP
using MibS_jll


function _build_single_model(
    model::BilevelModel,
)
    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)
    
    linkLU = Dict(

        # model.link means all link from upper to lower
        JuMP.index(v) => JuMP.index(k) for (k,v) in model.link

    )

    linkLOnly = Dict(
        JuMP.index(k) => JuMP.index(v) for (k, v) in model.lower_to_upper_link
    )
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

    
    lower_constraints = Vector{MOI.ConstraintIndex}()
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

    lower_sense = MOI.get(lower, MOI.ObjectiveSense())

    return model, lower_variables, lower_objective, lower_constraints, lower_sense
end


function index_to_row_link(
    model::MOI.FileFormats.MPS.Model)

    i = 1
    dict = Dict{MOI.ConstraintIndex, Int}()
    for (S, _) in MOI.FileFormats.MPS.SET_TYPES
        for ci in MOI.get(
            model,
            MOI.ListOfConstraintIndices{MOI.ScalarAffineFunction{Float64}, S}(),
        )  
            dict[ci] = i
            i += 1
        end
    end
    return dict
end


function index_to_column_link(
    model::MOI.FileFormats.MPS.Model)

    variables = MOI.get(model, MOI.ListOfVariableIndices())
    return Dict{MOI.VariableIndex,Int}(
        x => i - 1 for (i, x) in MOI.enumerate(variables)
    )
end


function write_auxillary_file(
    new_model::MOI.FileFormats.MPS.Model,
    lower_variables::Vector{MOI.VariableIndex},
    lower_objective::MOI.ScalarAffineFunction,
    lower_constraints::Vector{MOI.ConstraintIndex},
    lower_sense::MOI.OptimizationSense,
    aux_address::AbstractString
    )

    Row = index_to_row_link(new_model)
    Col = index_to_column_link(new_model)

    open(aux_address, "w") do io 
        println(io, "N    $(length(lower_variables))")
        println(io, "M    $(length(lower_constraints))")

        for x in lower_variables
            println(io, "LC    $(Col[x])")
        end

        for y in lower_constraints
            println(io, "LR    $(Row[y])")
        end
        
        obj_coefficients = Dict{MOI.VariableIndex,Float64}(
            x => 0.0 for x in lower_variables
        )
        
        for term in lower_objective.terms
            if haskey(obj_coefficients, term.variable_index)
                obj_coefficients[term.variable_index] += term.coefficient
            end
        end
        
        for x in lower_variables
            println(io, "LO    $(obj_coefficients[x])")
        end
        
        println(io, "OS    ", lower_sense == MOI.MAX_SENSE ? 1 : -1)
    end
end



function Writing_MibS_inputs(
    model::BilevelModel,
    name::AbstractString = "model"
)

    new_model, lower_variables, lower_objective, lower_constraints, lower_sense  = _build_single_model(model)

    # Write the MPS
    #name = "modelv2"
    name_mps = string(name, ".mps")
    name_aux = string(name, ".aux")

    MOI.write_to_file(new_model, name_mps)
    write_auxillary_file(new_model, lower_variables, lower_objective, lower_constraints, lower_sense, name_aux)

end

function Running_MibS(
add_mpx::AbstractString = "model.mps",
add_aux::AbstractString = "model.aux"
)

    MibS_jll.mibs() do exe
        run(`$(exe) -Alps_instance $(add_mpx) -MibS_auxiliaryInfoFile $(add_aux)`)
    end

end



function test_Writing_MibS_input_v1()

    model = BilevelModel()

    @variable(Upper(model), y)
    @variable(Upper(model), z)
    @variable(Lower(model), x)
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

    Writing_MibS_inputs(model, "modelv1")
    
end


function test_Writing_MibS_input_v2()

    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)
    @objective(Upper(model), Min, -3x - 7y)
    @constraints(Upper(model), begin
        u1, -3x + 2y <= 12
        u2, x + 2y <= 20
        u3, x <= 10
    end)

    @objective(Lower(model), Min, y)
    @constraint(Lower(model), l1,  2x -  y <= 7)
    @constraint(Lower(model), l2, -2x +  4y <= 16)
    @constraint(Lower(model), l3, y <= 5)

    Writing_MibS_inputs(model, "modelv2")
end

#test_Writing_MibS_input_v1()
#test_Writing_MibS_input_v2()

#Running_MibS("modelv1.mps", "modelv1.aux")
#Running_MibS("modelv2.mps", "modelv2.aux")
Running_MibS("modelv2.mps", "modelv2.aux")
