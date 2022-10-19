function JuMP.num_variables(m::AbstractBilevelModel)
    return JuMP.num_variables(bilevel_model(m))
end
_plural(n) = (isone(n) ? "" : "s")
function JuMP.show_constraints_summary(io::IO, model::BilevelModel)
    JuMP.show_constraints_summary(io, Upper(model))
    return JuMP.show_constraints_summary(io, Lower(model), true)
end
function JuMP.show_constraints_summary(io::IO, model::UpperModel)
    n = JuMP.num_constraints(model)
    return println(io, "Upper Constraint", _plural(n), ": ", n)
end
function JuMP.show_constraints_summary(
    io::IO,
    model::LowerModel,
    line_break = false,
)
    n = JuMP.num_constraints(model)
    return print(
        io,
        "Lower Constraint",
        _plural(n),
        ": ",
        n,
        ifelse(line_break, "\n", ""),
    )
end
function JuMP.show_backend_summary(io::IO, model::InnerBilevelModel)
    return JuMP.show_backend_summary(io, model.m)
end
function JuMP.show_backend_summary(io::IO, model::BilevelModel)
    println(io, "Bilevel Model")
    println(io, "Solution method: ", model.mode)
    if model.solver === nothing
        print(io, "No solver attached")
    else
        name = JuMP.solver_name(model::BilevelModel)
        print(io, "Solver name: ", name)
    end
end
function JuMP.solver_name(model::BilevelModel)
    name = try
        MOI.get(model.solver, MOI.SolverName())::String
    catch ex
        if isa(ex, ArgumentError)
            return "SolverName() attribute not implemented by the optimizer."
        else
            rethrow(ex)
        end
    end
end

function JuMP.show_objective_function_summary(io::IO, model::UpperModel)
    return println(
        io,
        "Upper objective function type: \n",
        JuMP.objective_function_type(model),
    )
end
function JuMP.show_objective_function_summary(io::IO, model::LowerModel)
    return println(
        io,
        "Lower objective function type: \n",
        JuMP.objective_function_type(model),
    )
end
function JuMP.show_objective_function_summary(io::IO, model::BilevelModel)
    JuMP.show_objective_function_summary(io, Upper(model))
    return JuMP.show_objective_function_summary(io, Lower(model))
end
