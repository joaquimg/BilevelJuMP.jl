
"""
Build the subproblem for valid inequalities
```
min_{x,v,λ} 0
            G x + H v ≤ q (upper level constraints)
            A x + B v ≤ b (lower level constraints)
            d + B^T λ ≥ 0 (dual lower level constraints)
            x ∈ X (upper level other constraints)
```
The objective `A_i ⋅ x` can then be set later.
"""
function build_inequality_subproblem_constraints(bilevel_model::AbstractBilevelModel; optimizer=nothing)

    m = Model()
    if optimizer !== nothing
        set_optimizer(model, optimizer)
    end
    # Add variables from both levels

    # Copy UL and LL constraints

    # Dualize lower-level model and add constraints

    return m
end

"""
Takes a bilevel model and the `submodel` built by `build_inequality_subproblem_constraints`,
sets the objective as `max_x func(cons)`, the function of the passed linear constraint of the form affine-function less than constant.
If the constraint is of the form: `a ⋅ x + b ⋅ y ≤ c`, this corresponds to `max_x a ⋅ x`.
The constraint must be a lower level one.
"""
function set_optimize_inequality_subproblem(bilevel_model::AbstractBilevelModel, submodel::JuMP.AbstractModel, cons::ConstraintRef{<:BilevelJuMP.AbstractBilevelModel})
    # TODO get vector of upper-level constraints
    x = nothing
    cobj = constraint_object(cons)
    objective_func = AffExpr()
    sign_multiplier = if cobj.set isa MOI.GreaterThan
        1
    elseif cobj.set isa MOI.GreaterThan
        sign_multiplier = -1
    else
        # equality not handled because value would be trivial / independent of x
        error("Constraint should be an inequality")
    end
    for (var, coeff) in cres.func.terms
        xi = nothing # TODO find variable corresponding to var in submodel
        add_to_expression!(objective_func, xi, sign_multiplier * coeff)
    end
    set_objective_function(submodel, objective_func)
    set_objective_sense(submodel, MOI.MIN_SENSE)
    return submodel
end

function compute_valid_inequalities(bilevel_model::AbstractBilevelModel; max_iterations=50, optimizer=nothing)
    submodel = build_inequality_subproblem_constraints(bilevel_model, optimizer=optimizer)
    # tuples with the constraint and the current count
    constraint_count = []
    for (F, S) in list_of_constraint_types(Lower(bilevel_model))
        if F <: GenericAffExpr
            for cons in all_constraints(Lower(bilevel_model), F, S)
                push!(constraint_count, (cons, 0))
            end
        end
    end
end
