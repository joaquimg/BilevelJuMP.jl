JuMP.num_variables(m::AbstractBilevelModel) = JuMP.num_variables(bilevel_model(m))
_plural(n) = (isone(n) ? "" : "s")
function JuMP.show_constraints_summary(io::IO, model::BilevelModel)
    JuMP.show_constraints_summary(io, Upper(model))
    JuMP.show_constraints_summary(io, Lower(model), true)
end
function JuMP.show_constraints_summary(io::IO, model::UpperModel)
    n = JuMP.num_constraints(model)
    println(io, "Upper Constraint", _plural(n), ": ", n)
end
function JuMP.show_constraints_summary(io::IO, model::LowerModel, line_break = false)
    n = JuMP.num_constraints(model)
    print(io, "Lower Constraint", _plural(n), ": ", n, ifelse(line_break, "\n", ""))
end
JuMP.show_backend_summary(io::IO, model::InnerBilevelModel) =
    JuMP.show_backend_summary(io, model.m)
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
    println(io, "Upper objective function type: \n", JuMP.objective_function_type(model))
end
function JuMP.show_objective_function_summary(io::IO, model::LowerModel)
    println(io, "Lower objective function type: \n", JuMP.objective_function_type(model))
end
function JuMP.show_objective_function_summary(io::IO, model::BilevelModel)
    JuMP.show_objective_function_summary(io, Upper(model))
    JuMP.show_objective_function_summary(io, Lower(model))
end
