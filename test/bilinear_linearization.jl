# using Logging; global_logger(ConsoleLogger(stderr, Logging.Debug))
#=
Testing linearization of upper objective bilinear terms of lower duals and primals and supporting
functions.

NOTE: test/jump.jl contains a test (jump_conejo2016_linearize) that confirms same results for a 
bilinear and linearized problem.
=#


function test_recursive_col_search()
    A = ones(2,2)
    try
        BilevelJuMP.recursive_col_search(A, 1, 1, Int[], Int[])
    catch e  # redundant rows
        @test typeof(e) == BilevelJuMP.UnderDeterminedException
    end
    A[1,2] = 0
    try
        BilevelJuMP.recursive_col_search(A, 1, 1, Int[], Int[2])
    catch e  # redundant cols
        @test typeof(e) == BilevelJuMP.UnderDeterminedException
    end
end


function test_find_connected_rows_cols()
    A = Matrix{Float64}([
        [1  0];
        [0  1]
    ])
    rows, cols, redundant_vals = BilevelJuMP.find_connected_rows_cols(A, 1, 1; 
        skip_1st_col_check=false, check_column_of_row=false
    )
    @test isempty(rows)
    @test cols == [1]
    @test redundant_vals == false

    A[2,1] = 1
    rows, cols, redundant_vals = BilevelJuMP.find_connected_rows_cols(A, 1, 1; 
        skip_1st_col_check=false, check_column_of_row=true
    )
    @test rows == cols

    A = ones(2,2)
    rows, cols, redundant_vals = BilevelJuMP.find_connected_rows_cols(A, 1, 1; 
        skip_1st_col_check=false, check_column_of_row=false
    )
    @test redundant_vals == true
end


function simple_linearization(optimizer, mode = BilevelJuMP.SOS1Mode())

    cder = 1    
    clmp = 1
    ci = 1
    d1 = 1
    d2 = 2
    MOI.empty!(optimizer)
    # MILP Program (bilinear terms in upper and lower objectives get linearized)
    model = BilevelModel(
        ()->optimizer, 
        mode = mode, 
        linearize_bilinear_upper_terms=true
    )
    @variables(Upper(model), begin
        10 >= x0 >= 0
        10 >= xe >= 0
    end)

    @variables(Lower(model), begin
        10 >= ye >= 0
        10 >= yi >= 0
        10 >= yder >= 0
    end)

    @objective(Lower(model), Min, cder * yder + ci * yi - xe * ye)
    @constraint(Lower(model), loadbal, yi - ye + yder == d1)

    @variable(Upper(model), lambda, DualOf(loadbal))

    @objective(Upper(model), Min, clmp * x0 + lambda * ye)
    @constraint(Upper(model), x0 + ye - yi - d2 == 0)
    @constraint(Upper(model), [ye, yi] in MOI.SOS1([1.0, 2.0]))

    optimize!(model)

    @expression(model, LLcost, cder * yder + ci * yi - xe * ye)
    @test objective_value(model) ≈ 2.0
    @test value(LLcost) ≈ 1.0
    @test value(yi) ≈ 0.0
    @test value(ye) ≈ 0.0
    @test value(yder) ≈ 1.0
    @test value(x0) ≈ 2.0

    @test MOI.get(model.solver, MOI.ObjectiveFunctionType()) == MathOptInterface.ScalarAffineFunction{Float64}

end


"""
make some dummy models that violate the linearization conditions when the set AB_N is non-empty, 
i.e. when there are xm*yn in the LL objective


for local test:
```julia
using BilevelJuMP, JuMP, Gurobi, Test
optimizer = Gurobi.Optimizer()
mode = BilevelJuMP.SOS1Mode()

```
"""
function failing_conditions_non_empty_AB_N(optimizer, mode = BilevelJuMP.SOS1Mode())

    cder = 1    
    clmp = 1
    ci = 1
    d1 = 1
    d2 = 2

    MOI.empty!(optimizer)

    model = BilevelModel(
        ()->optimizer, 
        mode = mode, 
        linearize_bilinear_upper_terms=true
    )
    @variables(Upper(model), begin
        10 >= x0 >= 0
        10 >= xe >= 0
        10 >= xbad1 >= 0
    end)

    @variables(Lower(model), begin
        10 >= ye >= 0
        10 >= yi >= 0
        10 >= yder >= 0
        10 >= ybad3 >= 0
        10 >= ybad4 >= 0
        10 >= ybad5 >= 0
    end)

    # condition 2: no connected variables * upper variable in LL obj.
    @objective(Lower(model), Min, cder * yder + ci * yi - xe * ye + 2*xe*yi)
 
    # @objective(Lower(model), Min, cder * yder + ci * yi - xe * ye)

    @constraint(Lower(model), loadbal, yi - ye + yder + ybad5 == d1)
    # need minus sign on ybad5 to meet condition 5
    
    # # adding xbad to connected constraint results in Condition 1 not being met
    @constraint(Lower(model), badcon1, yi + xbad1 == d1)

    #= 
    adding ye (lower variable in upper bilinear obj. terms) to another constraint violates 
        Condition 4 
    =#
    @constraint(Lower(model), badcon4, ye + ybad4 == d1)

    # conditon 3: ybad3 is not connected to other yn in bilinear upper obj. terms
    @constraint(Lower(model), badcon3, ybad3 + xbad1 == d1)

    @variable(Upper(model), lambda, DualOf(loadbal))
    @variable(Upper(model), dummylambda, DualOf(badcon3))

    # having different coef.s on ratio of obj. coef.s to constraint coef.s on the upper obj. 
    # bilinear terms violates Condition 5
    @objective(Upper(model), Min, clmp * x0 + lambda * ye + lambda * ybad5 - dummylambda * ybad3)
    @constraint(Upper(model), x0 + ye - yi - d2 == 0)
    @constraint(Upper(model), [ye, yi] in MOI.SOS1([1.0, 2.0]))

    # code from JuMP.optimize! in src/jump.jl
    upper = JuMP.backend(model.upper)
    lower = JuMP.backend(model.lower)
    lower_var_indices_of_upper_vars = JuMP.index.(collect(values(model.upper_to_lower_link)))
    upper_to_lower_var_indices = BilevelJuMP.convert_indices(model.link)
    upper_var_to_lower_ctr = BilevelJuMP.index2(model.upper_var_to_lower_ctr_link)
    BilevelJuMP.build_bounds!(model, mode)
    copy_names = false

    @test BilevelJuMP.is_model_in_standard_form(lower) == true

    # code from build_bilevel in src/moi.jl
    m, lower_dual, lower_primal_dual_map, upper_to_m_idxmap, lower_to_m_idxmap, lower_dual_idxmap = 
    BilevelJuMP.build_maps(
        upper, lower, upper_to_lower_var_indices, lower_var_indices_of_upper_vars, mode, 
        upper_var_to_lower_ctr, copy_names
    )

    # code from main_linearization in src/bilinear_linearization.jl
    A_N, bilinear_upper_dual_to_quad_term, bilinear_upper_dual_to_lower_primal, lower_primal_var_to_lower_con = 
    BilevelJuMP.check_upper_objective_for_bilinear_linearization(
        upper, upper_to_lower_var_indices, upper_var_to_lower_ctr
    )
    AB_N, B, lower_obj_terms, lower_obj_type_handled = 
    BilevelJuMP.get_lower_obj_coefs_of_upper_times_lower_primals(
        lower, lower_var_indices_of_upper_vars, A_N, lower_to_m_idxmap
    )
    U, V, w = BilevelJuMP.standard_form(lower, upper_var_indices=lower_var_indices_of_upper_vars)
    J_U, N_U = BilevelJuMP.get_all_connected_rows_cols(upper_var_to_lower_ctr, bilinear_upper_dual_to_lower_primal, V, AB_N)
    @test !isempty(AB_N)
    num_blocks, rows, cols = BilevelJuMP.find_blocks(V, U)

    @test B[2, 5] == 2

    @test !(BilevelJuMP.check_condition_1(J_U, U))

    @test !(BilevelJuMP.check_condition_2prime(N_U, A_N, U, B))

    @test !(BilevelJuMP.check_condition_3(A_N, V, lower_primal_var_to_lower_con))

    @test !(BilevelJuMP.check_condition_4(A_N, V, upper_var_to_lower_ctr, bilinear_upper_dual_to_lower_primal))

    @test !(BilevelJuMP.check_condition_5(A_N, V, upper_var_to_lower_ctr, 
        bilinear_upper_dual_to_lower_primal, bilinear_upper_dual_to_quad_term))

    # TODO test rest of conditions are not met

    # optimize!(model)
    # TODO need Xpress test for this code to run (then see if codecov shows the conditions being covered)
    # TODO break linearization code into more functions that can be tested in place of the optimize! pipeline

    # try
    #     optimize!(model)
    # catch e
    #     # will fail in Gurobi b/c ERROR: Gurobi Error 10020: Q matrix is not positive semi-definite (PSD)
    # end

end



# m = MOIU.CachingOptimizer(MOIU.UniversalFallback(MOIU.Model{Float64}()), MOIU.AUTOMATIC)
# using Dualization
# dual_problem = Dualization.dualize(lower,
#     dual_names = Dualization.DualNames("dual_","dual_"),
#     variable_parameters = lower_var_indices_of_upper_vars,
#     ignore_objective = BilevelJuMP.ignore_dual_objective(mode))
# # the model
# lower_dual = dual_problem.dual_model
# # the mapping from primal to dual references
# lower_primal_dual_map = dual_problem.primal_dual_map


# upper_to_m_idxmap = MOIU.default_copy_to(m, upper, false)

# BilevelJuMP.handle_lower_objective_sense(lower)

# lower_to_m_idxmap = MOIU.IndexMap()
# lower_to_upper_var_indices = Dict{MOI.VariableIndex, MOI.VariableIndex}()
# for (upper_key, lower_val) in upper_to_lower_var_indices
#     lower_to_m_idxmap[lower_val] = upper_to_m_idxmap[upper_key]
#     lower_to_upper_var_indices[lower_val] = upper_key
# end

# # append the second level primal
# BilevelJuMP.append_to(m, lower, lower_to_m_idxmap, false, allow_single_bounds = true)


# lower_dual_idxmap = MOIU.IndexMap()
# # for lower level QP's there are lower dual variables that are tied to:
# # lower primal variables
# for (lower_primal_var_key, lower_dual_quad_slack_val) in lower_primal_dual_map.primal_var_dual_quad_slack
#     lower_dual_idxmap[lower_dual_quad_slack_val] = lower_to_m_idxmap[lower_primal_var_key]
# end
# # and to upper level variables (which are lower level parameters)
# for (lower_primal_param_key, lower_dual_param_val) in lower_primal_dual_map.primal_parameter
#     lower_dual_idxmap[lower_dual_param_val] = lower_to_m_idxmap[lower_primal_param_key]
# end
# # lower level dual variable -> upper level variable
# # and the reverse map
# bilinear_upper_primal_lower_dual = MOIU.IndexMap()
# for (upper_var, lower_con) in upper_var_to_lower_ctr
#     var = lower_primal_dual_map.primal_con_dual_var[lower_con][1] # TODO check this scalar
#     lower_dual_idxmap[var] = upper_to_m_idxmap[upper_var]
#     bilinear_upper_primal_lower_dual[upper_to_m_idxmap[upper_var]] = var
# end

# # append the second level dual
# BilevelJuMP.append_to(m, lower_dual, lower_dual_idxmap, false)


# # main_linearization(
# #         m, lower, upper, upper_var_to_lower_ctr, upper_to_lower_var_indices, 
# #         lower_var_indices_of_upper_vars, lower_to_m_idxmap, upper_to_m_idxmap, 
# #         lower_primal_dual_map, lower_dual_idxmap
# #     )


# lower_con_types = Set(MOI.get(lower, MOI.ListOfConstraints()))
# standard_form_con_types = Set([
#     (MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64}),
#     (MOI.SingleVariable, MOI.GreaterThan{Float64}),
#     (MOI.SingleVariable, MOI.LessThan{Float64})
# ])



# A_N, bilinear_upper_dual_to_quad_term, bilinear_upper_dual_to_lower_primal, lower_primal_var_to_lower_con = 
#                 BilevelJuMP.check_upper_objective_for_bilinear_linearization(upper, upper_to_lower_var_indices, upper_var_to_lower_ctr)

# AB_N, B, lower_obj_terms, lower_obj_type_handled = 
# BilevelJuMP.get_lower_obj_coefs_of_upper_times_lower_primals(lower, lower_var_indices_of_upper_vars, A_N)

# U, V, w = BilevelJuMP.standard_form(lower, upper_var_indices=lower_var_indices_of_upper_vars)

# upper_obj_func_quad_terms = MOI.get(upper, MOI.ObjectiveFunction{MOI.get(upper, MOI.ObjectiveFunctionType())}()).quadratic_terms
# linearizations = nothing
# m_objective = MOI.get(m, MOI.ObjectiveFunction{MOI.get(m, MOI.ObjectiveFunctionType())}())
# bilinear_upper_quad_term_to_m_quad_term = Dict{MOI.ScalarQuadraticTerm, MOI.ScalarQuadraticTerm}()

# for term in m_objective.quadratic_terms
#     mset = Set([term.variable_index_1, term.variable_index_2])
#     for upper_term in upper_obj_func_quad_terms
#         uset = Set([
#             upper_to_m_idxmap[upper_term.variable_index_1],
#             upper_to_m_idxmap[upper_term.variable_index_2]
#         ])
#         if uset == mset
#             bilinear_upper_quad_term_to_m_quad_term[upper_term] = term
#         end
#     end
# end


# J_U = Int[]
# N_U = Int[]
# for (upper_var, lower_con) in upper_var_to_lower_ctr  # equivalent to set A with pairs (j,n) : A_jn ≠ 0
#     j = lower_con.value
#     n = bilinear_upper_dual_to_lower_primal[upper_var].value
#     if !(upper_var in keys(bilinear_upper_dual_to_lower_primal)) continue end  # user defined DualOf but did not use it in UL objective
#     rows, cols = BilevelJuMP.find_connected_rows_cols(V, j, n, skip_1st_col_check=!(isempty(AB_N)))
#     push!(J_U, rows...)
#     push!(N_U, cols...)
# end

#=  TODO
following results in 


ERROR: AssertionError: A[row, col] != 0
Stacktrace:
 [1] find_connected_rows_cols(A::SparseArrays.SparseMatrixCSC{Float64, Int64}, row::Int64, col::Int64; skip_1st_col_check::Bool, check_column_of_row::Bool)
   @ BilevelJuMP ~/Projects/BilevelJuMP.jl/src/bilinear_linearization.jl:101
 [2] check_non_empty_AB_N_conditions(J_U::Vector{Int64}, U::SparseArrays.SparseMatrixCSC{Float64, Int64}, N_U::Vector{Int64}, A_N::Vector{Int64}, B::SparseArrays.SparseMatrixCSC{Float64, Int64}, V::SparseArrays.SparseMatrixCSC{Float64, Int64}, lower_primal_var_to_lower_con::Dict{MathOptInterface.VariableIndex, MathOptInterface.ConstraintIndex}, upper_var_lower_ctr::Dict{MathOptInterface.VariableIndex, MathOptInterface.ConstraintIndex}, bilinear_upper_dual_to_quad_term::Dict{MathOptInterface.VariableIndex, MathOptInterface.ScalarQuadraticTerm}, bilinear_upper_dual_to_lower_primal::Dict{MathOptInterface.VariableIndex, MathOptInterface.VariableIndex})
   @ BilevelJuMP ~/Projects/BilevelJuMP.jl/src/bilinear_linearization.jl:724
 [3] main_linearization(m::MathOptInterface.Utilities.CachingOptimizer{MathOptInterface.AbstractOptimizer, MathOptInterface.Utilities.UniversalFallback{MathOptInterface.Utilities.GenericModel{Float64, MathOptInterface.Utilities.ModelFunctionConstraints{Float64}}}}, lower::MathOptInterface.Utilities.CachingOptimizer{MathOptInterface.AbstractOptimizer, MathOptInterface.Utilities.UniversalFallback{MathOptInterface.Utilities.GenericModel{Float64, MathOptInterface.Utilities.ModelFunctionConstraints{Float64}}}}, upper::MathOptInterface.Utilities.CachingOptimizer{MathOptInterface.AbstractOptimizer, MathOptInterface.Utilities.UniversalFallback{MathOptInterface.Utilities.GenericModel{Float64, MathOptInterface.Utilities.ModelFunctionConstraints{Float64}}}}, upper_var_to_lower_ctr::Dict{MathOptInterface.VariableIndex, MathOptInterface.ConstraintIndex}, upper_to_lower_var_indices::Dict{MathOptInterface.VariableIndex, MathOptInterface.VariableIndex}, lower_var_indices_of_upper_vars::Vector{MathOptInterface.VariableIndex}, lower_to_m_idxmap::MathOptInterface.Utilities.IndexMap, upper_to_m_idxmap::MathOptInterface.Utilities.IndexMap, lower_primal_dual_map::Dualization.PrimalDualMap{Float64}, lower_dual_idxmap::MathOptInterface.Utilities.IndexMap)
   @ BilevelJuMP ~/Projects/BilevelJuMP.jl/src/bilinear_linearization.jl:955
 [4] build_bilevel(upper::MathOptInterface.Utilities.CachingOptimizer{MathOptInterface.AbstractOptimizer, MathOptInterface.Utilities.UniversalFallback{MathOptInterface.Utilities.GenericModel{Float64, MathOptInterface.Utilities.ModelFunctionConstraints{Float64}}}}, lower::MathOptInterface.Utilities.CachingOptimizer{MathOptInterface.AbstractOptimizer, MathOptInterface.Utilities.UniversalFallback{MathOptInterface.Utilities.GenericModel{Float64, MathOptInterface.Utilities.ModelFunctionConstraints{Float64}}}}, upper_to_lower_var_indices::Dict{MathOptInterface.VariableIndex, MathOptInterface.VariableIndex}, lower_var_indices_of_upper_vars::Vector{MathOptInterface.VariableIndex}, mode::BilevelJuMP.SOS1Mode{Float64}, upper_var_to_lower_ctr::Dict{MathOptInterface.VariableIndex, MathOptInterface.ConstraintIndex}; copy_names::Bool, pass_start::Bool, linearize_bilinear_upper_terms::Bool)
   @ BilevelJuMP ~/Projects/BilevelJuMP.jl/src/moi.jl:422
 [5] optimize!(model::BilevelModel; lower_prob::String, upper_prob::String, bilevel_prob::String, solver_prob::String, file_format::MathOptInterface.FileFormats.FileFormat)
   @ BilevelJuMP ~/Projects/BilevelJuMP.jl/src/jump.jl:516
 [6] optimize!(model::BilevelModel)
   @ BilevelJuMP ~/Projects/BilevelJuMP.jl/src/jump.jl:475
 [7] top-level scope
   @ REPL[47]:1



MOI.empty!(optimizer)

model = BilevelModel(
    ()->optimizer, 
    mode = mode, 
    linearize_bilinear_upper_terms=true
)

@variables(Upper(model), begin
    10 >= x0 >= 0
    10 >= xe >= 0
    10 >= xbad1 >= 0
end)

@variables(Lower(model), begin
    10 >= ye >= 0
    10 >= yi >= 0
    10 >= yder >= 0
    10 >= ybad4 >= 0
end)

@objective(Lower(model), Min, cder * yder + ci * yi - xe * ye)


@constraint(Lower(model), loadbal, yi - ye + yder + xbad1 == d1)


@constraint(Lower(model), badcon1, ye + ybad4 + xbad1 == d1)
@variable(Upper(model), lambda, DualOf(loadbal))

@objective(Upper(model), Min, clmp * x0 + lambda * ye)


@constraint(Upper(model), x0 + ye - yi - d2 == 0)


@constraint(Upper(model), [ye, yi] in MOI.SOS1([1.0, 2.0]))


optimize!(model)
=#