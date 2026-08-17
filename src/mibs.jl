# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

# functionality for calling the MibS solver

function _build_single_model(
    model::BilevelModel,
    check_integrality::Bool = false,
)
    ret = _build_single_model_with_map(model, check_integrality)
    return ret[1], ret[2], ret[3], ret[4], ret[5]
end

# As `_build_single_model`, but also returns the map from the upper level
# variables to those of the merged model, which is what ties a solution
# reported by MibS back to the `BilevelModel`.
function _build_single_model_with_map(
    model::BilevelModel,
    check_integrality::Bool = false,
)
    _assert_mibs_supported(model)
    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)
    lower_to_upper =
        Dict(JuMP.index(v) => JuMP.index(k) for (k, v) in model.link)
    lower_only = Dict(
        JuMP.index(k) => JuMP.index(v) for (k, v) in model.lower_to_upper_link
    )
    return _build_single_model_with_map(
        upper,
        lower,
        lower_to_upper,
        lower_only,
        check_integrality,
    )
end

#=
    MibS reads a plain MPS file plus an auxiliary file listing which rows and
    columns belong to the lower level. Anything that cannot be expressed that way
    must be rejected up front, because MibS either crashes or silently solves a
    different problem than the one that was modeled.
=#

function _assert_mibs_supported(model::BilevelModel)
    if _has_nlp_data(model.upper) || _has_nlp_data(model.lower)
        error(
            "MibS does not support nonlinear data. Remove the `@NLobjective` " *
            "and `@NLconstraint` data from the model.",
        )
    end
    if !isempty(model.upper_var_to_lower_ctr_link)
        names = [JuMP.name(v) for v in keys(model.upper_var_to_lower_ctr_link)]
        error(
            "MibS does not support variables tied to lower level duals with " *
            "`DualOf`, but the model has: $(join(names, ", ")). MibS solves " *
            "the bilevel problem directly and never forms the dual of the " *
            "lower level, so such variables would be silently ignored.",
        )
    end
    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)
    for (level_model, level) in ((upper, "upper"), (lower, "lower"))
        _assert_supported_constraints(level_model, level)
        if MOI.get(level_model, MOI.ObjectiveSense()) == MOI.FEASIBILITY_SENSE
            error(
                "MibS requires an objective in the $(level) level. Set one " *
                "with `@objective($(titlecase(level))(model), Min, ...)`.",
            )
        end
        F = MOI.get(level_model, MOI.ObjectiveFunctionType())
        if !(F <: Union{MOI.VariableIndex,MOI.ScalarAffineFunction{Float64}})
            error(
                "MibS only supports linear objectives, but the $(level) level " *
                "objective has type $(F).",
            )
        end
    end
    return
end

function _assert_supported_constraints(model::MOI.ModelLike, level::String)
    for (F, S) in MOI.get(model, MOI.ListOfConstraintTypesPresent())
        if F === MOI.VariableIndex
            if !(
                S <: Union{
                    MOI.LessThan{Float64},
                    MOI.GreaterThan{Float64},
                    MOI.EqualTo{Float64},
                    MOI.Interval{Float64},
                    MOI.Integer,
                    MOI.ZeroOne,
                }
            )
                error(
                    "MibS does not support variables constrained to $(S), " *
                    "found in the $(level) level.",
                )
            end
        elseif F === MOI.ScalarAffineFunction{Float64}
            if !(
                S <: Union{
                    MOI.LessThan{Float64},
                    MOI.GreaterThan{Float64},
                    MOI.EqualTo{Float64},
                    MOI.Interval{Float64},
                }
            )
                error(
                    "MibS does not support linear constraints in $(S), " *
                    "found in the $(level) level.",
                )
            end
        else
            error(
                "MibS only supports linear constraints, but the $(level) " *
                "level has a constraint with function type $(F).",
            )
        end
    end
    return
end

function _build_single_model_with_map(
    upper::MOI.ModelLike,
    lower::MOI.ModelLike,
    lower_to_upper_link::Dict{MOI.VariableIndex,MOI.VariableIndex},
    lower_only::Dict{MOI.VariableIndex,MOI.VariableIndex},
    check_integrality::Bool = false,
)
    model = MOI.FileFormats.MPS.Model()
    upper_to_model_link = MOI.copy_to(model, upper)
    lower_variables = [upper_to_model_link[k] for k in values(lower_only)]
    function to_model(x)
        y = _lower_to_upper(lower, lower_to_upper_link, x)
        return upper_to_model_link[y]
    end
    lower_constraints = Vector{MOI.ConstraintIndex}()
    for (F, S) in MOI.get(lower, MOI.ListOfConstraintTypesPresent())
        for ci in MOI.get(lower, MOI.ListOfConstraintIndices{F,S}())
            lower_f = MOI.get(lower, MOI.ConstraintFunction(), ci)
            lower_s = MOI.get(lower, MOI.ConstraintSet(), ci)
            lower_f = MOI.Utilities.map_indices(to_model, lower_f)
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
    lower_objective = MOI.Utilities.map_indices(to_model, lower_objective)

    if check_integrality
        _assert_all_integer(model)
    end

    lower_sense = MOI.get(lower, MOI.ObjectiveSense())

    return model,
    lower_variables,
    lower_objective,
    lower_constraints,
    lower_sense,
    upper_to_model_link
end

# Every lower level variable must also be known to the upper level, which is the
# case for variables declared with `Lower(model)` or `Upper(model)` but not for
# those declared with `LowerOnly(model)`.
function _lower_to_upper(lower, lower_to_upper_link, x::MOI.VariableIndex)
    if !haskey(lower_to_upper_link, x)
        name = MOI.get(lower, MOI.VariableName(), x)
        error(
            "MibS does not support variables that belong only to the lower " *
            "level, but $(isempty(name) ? x : name) is one of them. Declare " *
            "it with `Lower(model)` instead of `LowerOnly(model)` so that it " *
            "is shared with the upper level.",
        )
    end
    return lower_to_upper_link[x]
end

function _assert_all_integer(model::MOI.FileFormats.MPS.Model)
    integer = Set{MOI.VariableIndex}()
    for S in (MOI.Integer, MOI.ZeroOne)
        for ci in
            MOI.get(model, MOI.ListOfConstraintIndices{MOI.VariableIndex,S}())
            push!(integer, MOI.get(model, MOI.ConstraintFunction(), ci))
        end
    end
    continuous = filter(
        x -> !(x in integer),
        MOI.get(model, MOI.ListOfVariableIndices()),
    )
    if !isempty(continuous)
        names = map(continuous) do x
            name = MOI.get(model, MOI.VariableName(), x)
            return isempty(name) ? string(x) : name
        end
        error(
            "MibS requires every variable to be integer, but the following " *
            "are continuous: $(join(names, ", ")). Note that MibS may run " *
            "forever instead of reporting an error when given a continuous " *
            "variable. Pass `check_integrality = false` to try anyway.",
        )
    end
    return
end

#=
    MibS identifies rows and columns by their position in the instance file, so the
    order in which the MPS writer emitted them is the ground truth. Recovering that
    order from the file itself keeps the auxiliary file correct even when
    MathOptInterface changes the order in which it emits constraints -- which has
    silently mislabeled the levels before.
=#

function _mps_row_and_column_order(mps_filename::String)
    rows, columns = String[], String[]
    seen = Set{String}()
    section = :none
    for line in eachline(mps_filename)
        isempty(strip(line)) && continue
        if !isspace(first(line))
            # Section headers are the only lines starting in the first column.
            keyword = uppercase(first(split(line)))
            section = if keyword == "ROWS"
                :rows
            elseif keyword == "COLUMNS"
                :columns
            else
                :other
            end
            continue
        end
        fields = split(line)
        if section == :rows
            # `<type> <name>`. The objective row is excluded from MibS's row
            # indexing, and it is the only row of type `N`.
            if length(fields) >= 2 && uppercase(fields[1]) != "N"
                push!(rows, fields[2])
            end
        elseif section == :columns
            # `<column> <row> <value> [<row> <value>]`, interleaved with the
            # `MARKER`/`INTORG`/`INTEND` lines that delimit integer columns.
            if !any(f -> occursin('\'', f), fields) && !(fields[1] in seen)
                push!(seen, fields[1])
                push!(columns, fields[1])
            end
        end
    end
    return rows, columns
end

function _mibs_index(order::Dict{String,Int}, name::String, what::String)
    if !haskey(order, name)
        error(
            "Unable to locate the $(what) \"$(name)\" in the MPS file written " *
            "for MibS. Please open an issue with BilevelJuMP.",
        )
    end
    return order[name]
end

function _write_auxiliary_file(
    new_model::MOI.FileFormats.MPS.Model,
    lower_variables::Vector{MOI.VariableIndex},
    lower_objective::MOI.ScalarAffineFunction,
    lower_constraints::Vector{MOI.ConstraintIndex},
    lower_sense::MOI.OptimizationSense,
    mps_filename::String,
    aux_filename::String,
)
    # `mps_filename` must already have been written: the writer assigns and
    # uniquifies the names that the auxiliary file refers to.
    row_names, column_names = _mps_row_and_column_order(mps_filename)
    rows = Dict(name => i - 1 for (i, name) in enumerate(row_names))
    cols = Dict(name => i - 1 for (i, name) in enumerate(column_names))
    function column_of(x)
        name = MOI.get(new_model, MOI.VariableName(), x)
        return _mibs_index(cols, name, "variable")
    end
    # MibS numbers the lower level variables within a block of their own, and
    # the lower objective coefficients are read in the same order as the `LC`
    # lines. Emitting them in ascending column order makes that block numbering
    # agree with the column order of the MPS file, which is what lets the values
    # MibS reports be matched back to variables.
    ordered_variables = sort(lower_variables; by = column_of)
    obj_coefficients =
        Dict{MOI.VariableIndex,Float64}(x => 0.0 for x in ordered_variables)
    for term in lower_objective.terms
        if haskey(obj_coefficients, term.variable)
            obj_coefficients[term.variable] += term.coefficient
        end
    end
    open(aux_filename, "w") do io
        println(io, "N $(length(ordered_variables))")
        println(io, "M $(length(lower_constraints))")
        for x in ordered_variables
            println(io, "LC $(column_of(x))")
        end
        for y in lower_constraints
            name = MOI.get(new_model, MOI.ConstraintName(), y)
            println(io, "LR $(_mibs_index(rows, name, "constraint"))")
        end
        for x in ordered_variables
            println(io, "LO $(obj_coefficients[x])")
        end
        return println(io, "OS ", lower_sense == MOI.MAX_SENSE ? -1 : 1)
    end
    return ordered_variables
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
        return run(
            pipeline(
                `$(exe) -Alps_instance $(mps_filename) -MibS_auxiliaryInfoFile $(aux_filename)`;
                stdout = io,
                stderr = io_err,
            ),
        )
    end
    # seekstart(io_err)
    # seekstart(io)
    return read(io, String), read(io_err, String)
end

function _parse_output(
    output::String,
    new_model::MOI.FileFormats.MPS.Model,
    lower_variables::Vector{MOI.VariableIndex},
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
        all_var = Dict_All,
    )
end

"""
    solve_with_MibS(model::BilevelModel, mibs_call; kwargs...)

## Inputs
* `model::BilevelModel`: the model to optimize
* `mibs_call`: should be `MibS_jll.mibs`, remember to `import MibS_jll` before.
* `verbose_results::Bool = false`: controls the verbosity of the solver output.
If `verbose_results=false`, nothing is printed.
Set to `true` to display the MibS output.
* `verbose_files::Bool = false`: Writes MibS input files to screen.
* `debug_file_prefix::String = ""`: Prefix prepended to the names of the MibS
input files saved to pwd() when `keep_files = true` or when MibS fails.
* `keep_files::Bool = false`: Saves MibS input files to pwd().
* `check_integrality::Bool = true`: Errors if any variable is continuous. MibS
may run forever instead of reporting an error on such a model, so this check is
on by default. Set to `false` to attempt the solve anyway.
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
function solve_with_MibS(
    model::BilevelModel,
    mibs_call;
    verbose_results::Bool = false,
    verbose_files::Bool = false,
    debug_file_prefix = "",
    keep_files::Bool = false,
    check_integrality::Bool = true,
)
    orig_path = pwd()
    mktempdir() do path
        mps_filename = joinpath(path, "model.mps")
        aux_filename = joinpath(path, "model.aux")
        new_model, variables, objective, constraints, sense =
            _build_single_model(model, check_integrality)
        # This MPS file must be strictly compliant with the format
        MOI.write_to_file(new_model, mps_filename)
        _write_auxiliary_file(
            new_model,
            variables,
            objective,
            constraints,
            sense,
            mps_filename,
            aux_filename,
        )
        if verbose_files
            @show mps_filename
            print(read(mps_filename, String))
            @show aux_filename
            print(read(aux_filename, String))
        end
        output, err = _call_mibs(mps_filename, aux_filename, mibs_call)
        if length(err) > 0 || keep_files
            mps_db = joinpath(orig_path, debug_file_prefix * "model.mps")
            aux_db = joinpath(orig_path, debug_file_prefix * "model.aux")
            try
                cp(mps_filename, mps_db; force = true)
            catch e
                println(
                    "BilevelJuMP failed to write debug file $mps_db: with $e",
                )
            end
            try
                cp(aux_filename, aux_db; force = true)
            catch e
                println(
                    "BilevelJuMP failed to write debug file $aux_db: with $e",
                )
            end
        end
        if length(err) > 0
            mibs_error =
                "MibS returned:\n\n" *
                "$err\n\n" *
                "MibS input files can be found at:\n" *
                "* $mps_db\n" *
                "* $aux_db\n\n" *
                "Please include these files if you open an issue.\n"
            error(mibs_error)
        end
        if length(output) == 0
            error("MibS failed to return")
        end
        if verbose_results
            println(output)
        end
        return _parse_output(output, new_model, variables)
    end
end

#=
    Solving through `MibSMode`.

    MibS is an external executable rather than a MathOptInterface optimizer, so
    the results cannot live in `model.solver` like they do for every other mode.
    They are collected here into a `MibSSolution` stored on the model, and the
    JuMP accessors are routed to it by dispatching on the mode.
=#

mutable struct MibSSolution
    termination_status::MOI.TerminationStatusCode
    primal_status::MOI.ResultStatusCode
    raw_status::String
    objective_value::Float64
    # keyed by `BilevelVariableRef.idx`
    primal::Dict{Int,Float64}
end

function _mibs_solution(model::BilevelModel)
    if model.solution === nothing
        error(
            "No solution available: call `optimize!(model)` before querying " *
            "results.",
        )
    end
    return model.solution::MibSSolution
end

# MibS reports values as `x[i]` for the upper level block and `y[i]` for the
# lower level block, numbered from zero within each block, and only after it
# announces an optimal solution.
function _parse_mibs_solution(output::AbstractString, upper, lower)
    lines = split(output, '\n')
    start = findfirst(l -> occursin("Optimal solution", l), lines)
    values = Dict{MOI.VariableIndex,Float64}()
    if start === nothing
        last_line = findlast(l -> !isempty(strip(l)), lines)
        raw = last_line === nothing ? "" : strip(lines[last_line])
        return MOI.OTHER_ERROR, MOI.NO_SOLUTION, String(raw), values
    end
    for v in vcat(upper, lower)
        values[v] = 0.0
    end
    for line in lines[(start+1):end]
        m = match(r"([xy])\[([0-9]+)\] *= *(.+)", line)
        m === nothing && continue
        block = m[1] == "x" ? upper : lower
        i = parse(Int, m[2]) + 1
        if 1 <= i <= length(block)
            values[block[i]] = parse(Float64, strip(m[3]))
        end
    end
    return MOI.OPTIMAL, MOI.FEASIBLE_POINT, String(strip(lines[start])), values
end

function _optimize!(model::BilevelModel, mode::MibSMode; kwargs...)
    if mode.mibs_call === nothing
        error(
            "No MibS executable was given to `MibSMode`. MibS is not a " *
            "dependency of BilevelJuMP: install and load `MibS_jll`, then " *
            "build the mode with `BilevelJuMP.MibSMode(MibS_jll.mibs)`.",
        )
    end
    if model.solver !== nothing
        error(
            "`MibSMode` solves the problem with the external MibS executable " *
            "and cannot use an optimizer attached with `set_optimizer`.",
        )
    end
    model.solution = nothing
    model.build_time = NaN
    model.solve_time = NaN
    t0 = time()
    return mktempdir() do path
        mps_filename = joinpath(path, "model.mps")
        aux_filename = joinpath(path, "model.aux")
        new_model,
        lower_variables,
        lower_objective,
        lower_constraints,
        lower_sense,
        upper_to_model =
            _build_single_model_with_map(model, mode.check_integrality)
        # This MPS file must be strictly compliant with the format
        MOI.write_to_file(new_model, mps_filename)
        ordered_lower = _write_auxiliary_file(
            new_model,
            lower_variables,
            lower_objective,
            lower_constraints,
            lower_sense,
            mps_filename,
            aux_filename,
        )
        t1 = time()
        model.build_time = t1 - t0
        output, err = _call_mibs(mps_filename, aux_filename, mode.mibs_call)
        model.solve_time = time() - t1
        if length(err) > 0
            error("MibS returned:\n\n$(err)\n")
        end
        if length(output) == 0
            error("MibS failed to return")
        end
        if mode.verbose
            println(output)
        end
        _, columns = _mps_row_and_column_order(mps_filename)
        lower_set = Set(ordered_lower)
        by_name = Dict(
            MOI.get(new_model, MOI.VariableName(), v) => v for
            v in MOI.get(new_model, MOI.ListOfVariableIndices())
        )
        ordered_upper =
            [by_name[name] for name in columns if !(by_name[name] in lower_set)]
        status, primal_status, raw, values =
            _parse_mibs_solution(output, ordered_upper, ordered_lower)
        primal = Dict{Int,Float64}()
        for (idx, v) in model.var_upper
            vi = JuMP.index(v)
            if haskey(upper_to_model, vi) && haskey(values, upper_to_model[vi])
                primal[idx] = values[upper_to_model[vi]]
            end
        end
        model.solution = MibSSolution(status, primal_status, raw, NaN, primal)
        if status == MOI.OPTIMAL
            # Evaluated rather than read from the MibS log, which reports the
            # objective of the MPS file. That file carries a negated objective
            # when the upper level is a maximization.
            model.solution.objective_value =
                JuMP.value(JuMP.objective_function(Upper(model)))
        end
        return nothing
    end
end

function _termination_status(model::BilevelModel, ::MibSMode)
    return _mibs_solution(model).termination_status
end

function _primal_status(model::BilevelModel, ::MibSMode)
    return _mibs_solution(model).primal_status
end

function _raw_status(model::BilevelModel, ::MibSMode)
    return _mibs_solution(model).raw_status
end

function _solver_name(::BilevelModel, ::MibSMode)
    return "MibS"
end

function _objective_value(model::BilevelModel, ::MibSMode)
    return _mibs_solution(model).objective_value
end

function _value(v::BilevelVariableRef, ::MibSMode; result::Int = 1)::Float64
    if result != 1
        error("MibS only provides a single solution.")
    end
    primal = _mibs_solution(owner_model(v)).primal
    if !haskey(primal, v.idx)
        error("No value available for $(v).")
    end
    return primal[v.idx]
end

# There is no solver to ask for a constraint primal, so evaluate the constraint
# function at the solution instead.
function _value(cref::BilevelConstraintRef, ::MibSMode; result::Int = 1)
    if result != 1
        error("MibS only provides a single solution.")
    end
    model = cref.model
    level = _in_upper(cref) ? Upper(model) : Lower(model)
    func = JuMP.constraint_object(_raw_ref(cref)).func
    return JuMP.value(_reverse_replace_variable(func, level))
end

function _dual(::BilevelConstraintRef, ::MibSMode)
    return error(
        "Dual solutions are not available when solving with " *
        "`BilevelJuMP.MibSMode`: MibS solves the bilevel problem directly and " *
        "never forms the dual of the lower level.",
    )
end
