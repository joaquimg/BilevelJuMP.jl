# using Logging; global_logger(ConsoleLogger(stderr, Logging.Debug))
#=
Testing linearization of upper objective bilinear terms of lower duals and primals and supporting
functions.

NOTE: test/jump.jl contains a test (jump_conejo2016_linearize) that confirms same results for a 
bilinear and linearized problem.
=#


function test_recursive_col_search()
    A = ones(2,2)  # redundant rows
    @test_throws BilevelJuMP.UnderDeterminedException BilevelJuMP.recursive_col_search(A, 1, 1, Int[], Int[])
    
    A[1,2] = 0  # redundant cols
    @test_throws BilevelJuMP.UnderDeterminedException BilevelJuMP.recursive_col_search(A, 1, 1, Int[], Int[2])
end


function test_find_connected_rows_cols()
    A = Matrix{Float64}([
        [1  0];
        [0  1]
    ])
    rows, cols, redundant_vals = BilevelJuMP.find_connected_rows_cols(A, 1, 1; 
        skip_1st_col_check=false, finding_blocks=false
    )
    @test isempty(rows)
    @test cols == [1]
    @test redundant_vals == false

    A[2,1] = 1
    rows, cols, redundant_vals = BilevelJuMP.find_connected_rows_cols(A, 1, 1; 
        skip_1st_col_check=false, finding_blocks=true
    )
    @test rows == cols

    A = ones(2,2)
    rows, cols, redundant_vals = BilevelJuMP.find_connected_rows_cols(A, 1, 1; 
        skip_1st_col_check=false, finding_blocks=false
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
function failing_conditions_non_empty_AB_N(mode = BilevelJuMP.SOS1Mode())

    cder = 1    
    clmp = 1
    ci = 1
    d1 = 1
    d2 = 2

    model = BilevelModel()
    model.linearize_bilinear_upper_terms = true
    
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
        upper, 
        lower, 
        upper_to_lower_var_indices, 
        lower_var_indices_of_upper_vars, 
        mode, 
        upper_var_to_lower_ctr, 
        copy_names
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

    # have to have function calls outside of @test for codecov to consider them tested?
    con1 = BilevelJuMP.check_condition_1(J_U, U)
    @test !(con1)
    con2 = BilevelJuMP.check_condition_2prime(N_U, A_N, U, B)
    @test !(con2)
    con3 = BilevelJuMP.check_condition_3(A_N, V, lower_primal_var_to_lower_con)
    @test !(con3)
    con4 = BilevelJuMP.check_condition_4(A_N, V, upper_var_to_lower_ctr, bilinear_upper_dual_to_lower_primal)
    @test !(con4)
    con5 = BilevelJuMP.check_condition_5(A_N, V, upper_var_to_lower_ctr, 
    bilinear_upper_dual_to_lower_primal, bilinear_upper_dual_to_quad_term)
    @test !(con5)

    BilevelJuMP.main_linearization(
        m,
        lower, 
        upper, 
        upper_var_to_lower_ctr, 
        upper_to_lower_var_indices, 
        lower_var_indices_of_upper_vars, 
        lower_to_m_idxmap, 
        upper_to_m_idxmap,
        lower_primal_dual_map, 
        lower_dual_idxmap,
        true
    )
    check_false = BilevelJuMP.check_non_empty_AB_N_conditions(J_U, U, N_U, A_N, B, V, lower_primal_var_to_lower_con, 
    upper_var_to_lower_ctr, bilinear_upper_dual_to_quad_term, bilinear_upper_dual_to_lower_primal)
    @test check_false == false
    linearizations = 
    BilevelJuMP.linear_terms_for_non_empty_AB(
        lower,
        upper_var_to_lower_ctr,
        bilinear_upper_dual_to_lower_primal,
        V,
        w,
        A_N,
        bilinear_upper_dual_to_quad_term,
        lower_obj_terms,
        lower_to_m_idxmap,
        lower_primal_dual_map,
        lower_dual_idxmap,
        false,
        num_blocks,
        rows
    )
end


function failing_conditions_empty_AB_N(mode = BilevelJuMP.SOS1Mode())
    
    cder = 1    
    clmp = 1
    ci = 1
    d1 = 1
    d2 = 2

    model = BilevelModel()
    model.linearize_bilinear_upper_terms = true

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

    @objective(Lower(model), Min, cder * yder + ci * yi)
    @constraint(Lower(model), loadbal, yi - ye + yder + xbad1 == d1)
    @constraint(Lower(model), badcon1, ye + ybad4 + xbad1 == d1)

    @variable(Upper(model), lambda, DualOf(loadbal))

    @objective(Upper(model), Min, clmp * x0 + lambda * ye)
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
        upper, 
        lower, 
        upper_to_lower_var_indices, 
        lower_var_indices_of_upper_vars, 
        mode, 
        upper_var_to_lower_ctr, 
        copy_names
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
    num_blocks, rows, cols = BilevelJuMP.find_blocks(V, U)

    @test isempty(AB_N)
    @test num_blocks == 1
    @test rows == [[1, 2]]
    @test cols == [[3, 4, 5, 6, 7]]
    @test !BilevelJuMP.check_empty_AB_N_conditions(J_U, U, N_U, B)
    @test_throws BilevelJuMP.UnderDeterminedException BilevelJuMP.recursive_col_search(U+V, 1, 3, Int[], Int[])

    # to reach more code
    @constraint((Lower(model)), ybad4 <= ye)
    @constraint((Lower(model)), yder >= ye)
    @constraint((Lower(model)), ybad4 <= 2)
    @constraint((Lower(model)), yder >= 1)
    U, V, w = BilevelJuMP.standard_form(lower, upper_var_indices=lower_var_indices_of_upper_vars)

end

function test_get_coef_matrix_and_rhs_vec()
    model = BilevelModel()

    @variable(Upper(model), x)
    @variable(Lower(model), y)

    @constraint(Lower(model), 2x + 3y <= 4)
    @constraint(Lower(model), 5x + 6y >= 7)
    lower = JuMP.backend(model.lower)
    con_indices = MOI.get(lower, MOI.ListOfConstraintIndices{
        MOI.ScalarAffineFunction{Float64}, 
        MOI.LessThan{Float64}
    }())
    A, b = BilevelJuMP.get_coef_matrix_and_rhs_vec(lower, con_indices)
    @test A == [2 3.0]
    @test b == [4.0]

    con_indices = MOI.get(lower, MOI.ListOfConstraintIndices{
        MOI.ScalarAffineFunction{Float64}, 
        MOI.GreaterThan{Float64}
    }())
    A, b = BilevelJuMP.get_coef_matrix_and_rhs_vec(lower, con_indices)
    @test A == [5 6.0]
    @test b == [7.0]
end
