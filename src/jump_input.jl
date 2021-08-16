

function _build_single_model(model::BilevelModel)
    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)
    lower_to_upper =
        Dict(JuMP.index(v) => JuMP.index(k) for (k, v) in model.link)
    lower_only = Dict(
        JuMP.index(k) => JuMP.index(v) for (k, v) in model.lower_to_upper_link
    )
    return _build_single_model(upper, lower, lower_to_upper, lower_only)
end

function _build_single_model(
    upper::MOI.ModelLike,
    lower::MOI.ModelLike,
    lower_to_upper_link::Dict{MOI.VariableIndex,MOI.VariableIndex},
    lower_only::Dict{MOI.VariableIndex,MOI.VariableIndex}
)
    model = MOI.FileFormats.MPS.Model()
    upper_to_model_link = MOI.copy_to(model, upper)
    lower_variables = [upper_to_model_link[k] for k in values(lower_only)]
    lower_constraints = Vector{MOI.ConstraintIndex}()
    for (F, S) in MOI.get(lower, MOI.ListOfConstraints())
        for ci in MOI.get(lower, MOI.ListOfConstraintIndices{F,S}())
            lower_f = MOI.get(lower, MOI.ConstraintFunction(), ci)
            lower_s = MOI.get(lower, MOI.ConstraintSet(), ci)
            lower_f = MOI.Utilities.map_indices(lower_f) do x
                return upper_to_model_link[lower_to_upper_link[x]]
            end
            new_ci = MOI.add_constraint(model, lower_f, lower_s)
            if F == MOI.ScalarAffineFunction{Float64}
                push!(lower_constraints, new_ci)
            end
        end
    end
    lower_objective = MOI.get(
        lower,
        MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
    )
    lower_objective = MOI.Utilities.map_indices(lower_objective) do x
        return upper_to_model_link[lower_to_upper_link[x]]
    end
    #number varibles == inter + binary const
    lower_sense = MOI.get(lower, MOI.ObjectiveSense())
    return model, lower_variables, lower_objective, lower_constraints, lower_sense
end

function _index_to_row_link(model::MOI.FileFormats.MPS.Model)
    i = 0
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

function _index_to_column_link(model::MOI.FileFormats.MPS.Model)
    variables = MOI.get(model, MOI.ListOfVariableIndices())
    return Dict{MOI.VariableIndex,Int}(
        x => i - 1 for (i, x) in MOI.enumerate(variables)
    )
end

function _write_auxillary_file(
    new_model::MOI.FileFormats.MPS.Model,
    lower_variables::Vector{MOI.VariableIndex},
    lower_objective::MOI.ScalarAffineFunction,
    lower_constraints::Vector{MOI.ConstraintIndex},
    lower_sense::MOI.OptimizationSense,
    aux_filename::String
)
    rows = _index_to_row_link(new_model)
    cols = _index_to_column_link(new_model)
    obj_coefficients = Dict{MOI.VariableIndex,Float64}(
        x => 0.0 for x in lower_variables
    )
    for term in lower_objective.terms
        if haskey(obj_coefficients, term.variable_index)
            obj_coefficients[term.variable_index] += term.coefficient
        end
    end
    open(aux_filename, "w") do io
        println(io, "N $(length(lower_variables))")
        println(io, "M $(length(lower_constraints))")
        for x in lower_variables
            println(io, "LC $(cols[x])")
        end
        for y in lower_constraints
            println(io, "LR $(rows[y])")
        end
        for x in lower_variables
            println(io, "LO $(obj_coefficients[x])")
        end
        println(io, "OS ", lower_sense == MOI.MAX_SENSE ? -1 : 1)
    end
    return
end

function _call_mibs(mps_filename, aux_filename)
    io = IOBuffer()
    MibS_jll.mibs() do exe
        run(
            pipeline(
                `$(exe) -Alps_instance $(mps_filename) -MibS_auxiliaryInfoFile $(aux_filename)`,
                stdout = io,
            )
        )
    end
    seekstart(io)
    return read(io, String)
end

function _parse_output(
    output::String,
    new_model::MOI.FileFormats.MPS.Model,
    lower_variables::Vector{MOI.VariableIndex}
    )
    lines = split(output, '\n')
    found_status = false
    objective_value = NaN

    upper = Dict{Int,Float64}()
    lower = Dict{Int,Float64}()


    all_var = MOI.get(new_model, MOI.ListOfVariableIndices())

    CntU = 0
    CntD = 0

    Dict_Lower_Name = Dict()
    Dict_Lower_Value = Dict()
    Dict_Upper_Name = Dict()
    Dict_Upper_Value = Dict()

    for (x, y) in MOI.enumerate(all_var)
        nameofvar = MOI.get(new_model, MOI.VariableName(), y)
        if y in lower_variables
            Dict_Lower_Name[CntD] = nameofvar
            Dict_Lower_Value[nameofvar] = 0
            CntD = CntD + 1
        else
            Dict_Upper_Name[CntU] = nameofvar  
            Dict_Upper_Value[nameofvar] = 0
            CntU = CntU + 1
        end
    end



    for line in lines
        if !found_status
            if occursin("Optimal solution", line)
                found_status = true
            end
            continue
        end
        m = match(r"([xy])\[([0-9]+)\] \= (.+)", line)
        if m === nothing
            m = match(r"Cost \= (.+)", line)
            if m !== nothing
                objective_value = parse(Float64, m[1])
            end
            continue
        end

        column = parse(Int, m[2])
        value = parse(Float64, m[3])

        if m[1] == "x"
            upper[column] = value
            nameofvar = Dict_Upper_Name[column]
            Dict_Upper_Value[nameofvar] = value
        else
            lower[column] = value
            nameofvar = Dict_Lower_Name[column]
            Dict_Lower_Value[nameofvar] = value
        end
    end

    return (
        status = found_status,
        objective = objective_value,
        nonzero_upper = upper,
        nonzero_lower = lower,
        all_upper = Dict_Upper_Value,
        all_lower = Dict_Lower_Value
    )
end

# input
# fails on ,...
# outputs

function solve_with_MibS(model::BilevelModel; silent::Bool = true)
    mktempdir() do path
        mps_filename = joinpath(path, "model.mps")
        aux_filename = joinpath(path, "model.aux")
        new_model, variables, objective, constraints, sense  =
            _build_single_model(model)
        MOI.write_to_file(new_model, mps_filename)
        _write_auxillary_file(
            new_model,
            variables,
            objective,
            constraints,
            sense,
            aux_filename,
        )
        output = _call_mibs(mps_filename, aux_filename)
        if silent
            println(output)
        end
        return _parse_output(output, new_model, variables)
    end
end
