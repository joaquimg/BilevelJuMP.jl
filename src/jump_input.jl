# functionality for calling the MibS solver

function _build_single_model(model::BilevelModel, check_MIPMIP::Bool = false)
    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)
    lower_to_upper =
        Dict(JuMP.index(v) => JuMP.index(k) for (k, v) in model.link)
    lower_only = Dict(
        JuMP.index(k) => JuMP.index(v) for (k, v) in model.lower_to_upper_link
    )
    return _build_single_model(upper, lower, lower_to_upper, lower_only, check_MIPMIP)
end

function _build_single_model(
    upper::MOI.ModelLike,
    lower::MOI.ModelLike,
    lower_to_upper_link::Dict{MOI.VariableIndex,MOI.VariableIndex},
    lower_only::Dict{MOI.VariableIndex,MOI.VariableIndex},
    check_MIPMIP::Bool = false
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

    
    # Testing if the model is MIP-MIP or not. 
    if check_MIPMIP
        int_var = MOI.get(model, MOI.NumberOfConstraints{MOI.SingleVariable, MOI.Integer}())
        int_var = int_var + MOI.get(model, MOI.NumberOfConstraints{MOI.SingleVariable, MOI.ZeroOne}())
        all_var = MOI.get(model, MOI.NumberOfVariables())
        if int_var != all_var
            throw("Currently MibS works on only MIP-MIP problems and the input model is not MIP-MIP!!")
        end
    end

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

function _call_mibs(mps_filename, aux_filename, mibs_call)
    #=
    MibS fail randomly in win ci if io = IOBuffer()
    writing to file has shown to be more robust
    =#
    io = "mibs_output.txt"
    # write(io, "\n BilevelJuMP Calling MibS \n")
    io_err = "mibs_errors.txt"
    mibs_call() do exe
        run(
            pipeline(
                `$(exe) -Alps_instance $(mps_filename) -MibS_auxiliaryInfoFile $(aux_filename)`,
                stdout = io,
                stderr = io_err,
            )
        )
    end
    # seekstart(io_err)
    err = read(io_err, String)
    if length(err) > 0
        error(err)
    end
    # seekstart(io)
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
    Dict_Upper_IndexToModel = Dict()
    Dict_Lower_IndexToModel = Dict()
    Dict_All = Dict()

    for (x, y) in MOI.enumerate(all_var)
        nameofvar = MOI.get(new_model, MOI.VariableName(), y)
        if y in lower_variables
            Dict_Lower_Name[CntD] = nameofvar
            Dict_Lower_Value[nameofvar] = 0
            Dict_Lower_IndexToModel[CntD] = y
            CntD = CntD + 1
        else
            Dict_Upper_Name[CntU] = nameofvar  
            Dict_Upper_Value[nameofvar] = 0
            Dict_Upper_IndexToModel[CntU] = y
            CntU = CntU + 1
        end
        Dict_All[y] = 0
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
            indexofvar = Dict_Upper_IndexToModel[column]
            Dict_All[indexofvar] = value
        else
            lower[column] = value
            nameofvar = Dict_Lower_Name[column]
            Dict_Lower_Value[nameofvar] = value
            indexofvar = Dict_Lower_IndexToModel[column]
            Dict_All[indexofvar] = value
        end
    end

    return (
        status = found_status,
        objective = objective_value,
        nonzero_upper = upper,
        nonzero_lower = lower,
        all_upper = Dict_Upper_Value,
        all_lower = Dict_Lower_Value,
        all_var = Dict_All
    )
end

"""
    solve_with_MibS(model::BilevelModel, mibs_call; silent::Bool = true)

## Inputs
* `model::BilevelModel`: the model to optimize
* `mibs_call`: shoul be `MibS_jll.mibs` remember to `import MibS_jll` before.
* `silent::Bool = true`: controls the verbosity of the solver. If `silent`, nothing is printed. Set to `false` to display the MibS output.
## Outputs
This function returns a `NamedTuple` with fields:
* `status::Bool`: `true` if the problem is feasible and has an optimal solution. `false` otherwise.
* `objective::Float64`: objective value (cost) of the upper problem
* `nonzero_upper::Dict{Int, Float64}`: it returns `Dict{index => value}`, in which the `index` refers to the index of upper variables with non zero values and the index starts from `0`. Here, the order of the variables is based on their order of appearance in the MPS file.
* `nonzero_lower::Dict{Int, Float64}`: it has the same structure as `nonzero_upper`, but it represents the index of non-zero variables in the lower problem. 
* `all_upper::Dict{String, Float64}`: it returns `Dict{name => value}` which contains all upper variables values (zero and non-zero). For recalling the variables, you need to use the same name as you used to define the variables, e.g., for `@variable(Upper(model), y, Int)`, we need to use `all_upper["y"]` to get the value of the variable `y`.
* `all_lower::Dict{String, Float64}`: it has the same structure as the `all_upper` but is defined for lower variables.
* `all_var::Dict{MOI.VariableIndex, Float64}`: it contains information on all variables (upper and lower) in the format of `MOI.VariableIndex` and their output values. 

!!! warning
    Currently, `MibS` is designed to solve MIP-MIP problems only. Thus, if you define LP-MIP, MIP-LP, or LP-LP, it will throw an error. 
"""
function solve_with_MibS(model::BilevelModel, mibs_call; silent::Bool = true, verbose_file::Bool = false)
    mktempdir() do path
        path = pwd()
        mps_filename = joinpath(path, "model.mps")
        aux_filename = joinpath(path, "model.aux")
        new_model, variables, objective, constraints, sense  =
            _build_single_model(model, true)
        MOI.write_to_file(new_model, mps_filename)
        _write_auxillary_file(
            new_model,
            variables,
            objective,
            constraints,
            sense,
            aux_filename,
        )
        if verbose_file
            @show mps_filename
            print(read(mps_filename, String))
            @show aux_filename
            print(read(aux_filename, String))
        end
        output = _call_mibs(mps_filename, aux_filename, mibs_call)
        if verbose_file
            print("MibS done")
        end
        if length(output) == 0
            error("MibS failed to return")
        end
        if !silent
            println(output)
        end
        return _parse_output(output, new_model, variables)
    end
end
