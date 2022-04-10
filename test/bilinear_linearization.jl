# global_logger(ConsoleLogger(stderr, Logging.Debug))

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


function bilinear_linearization(optimizer, mode = BilevelJuMP.SOS1Mode(), config = Config())

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

    # # MIQP
    # model2 = BilevelModel(Xpress.Optimizer, mode = BilevelJuMP.SOS1Mode())
    # @variables(Upper(model2), begin
    #     10 >= x0 >= 0
    #     10 >= xe >= 0
    # end)

    # @variables(Lower(model2), begin
    #     10 >= ye
    #     10 >= yi
    #     10 >= yder
    # end)

    # @constraints(Lower(model2), begin
    #     ye >= 0
    #     yi >= 0
    #     yder >= 0
    # end)

    # @objective(Lower(model2), Min, cder * yder + ci * yi - xe * ye)
    # @constraint(Lower(model2), loadbal, yi - ye + yder == d1)

    # @variable(Upper(model2), lambda, DualOf(loadbal))

    # @objective(Upper(model2), Min, clmp * x0 + lambda * ye)
    # @constraint(Upper(model2), x0 + ye - yi - d2 == 0)
    # @constraint(Upper(model2), [ye, yi] in MOI.SOS1([1.0, 2.0]))

    # optimize!(model2)

    # @expression(model2, LLcost, cder * yder + ci * yi - xe * ye)
    # @test objective_value(model2) ≈ 2.0
    # @test value(LLcost) ≈ 1.0
    # @test value(yi) ≈ 0.0
    # @test value(ye) ≈ 0.0
    # @test value(yder) ≈ 1.0
    # @test value(x0) ≈ 2.0


    #= 
            explicit KKT LL with linearized UL:
    =# 

    # model = JuMP.Model(Xpress.Optimizer)
    # set_optimizer_attribute(model, "NonConvex", 2)
    # set_optimizer_attribute(model, "MIPGap", 1e-2)

    # @variables(model, begin
    #     10 >= x0 >= 0
    #     10 >= xe >= 0
    #     # 10 >= xi >= -10
    #     10 >= ye >= 0
    #     10 >= yi >= 0
    #     10 >= yder >= 0
    #     10 >= lambda >= -10
    #     10 >= mu_e_up >= 0
    #     10 >= mu_e_lo >= 0
    #     10 >= mu_i_up >= 0
    #     10 >= mu_i_lo >= 0
    #     10 >= mu_der_up >= 0
    #     10 >= mu_der_lo >= 0
    # end)

    # # @objective(model, Min, clmp * x0 ) + lambda * ye)
    # ## bilinear problem results in costs UL=2, LL=1 and xe = 1, yder = 1, x0 = 2

    # @objective(model, Min, clmp * x0 + cder * yder + 10 * mu_der_up + ci * yi + 10 * mu_i_up - d1 * lambda)
    # ## linearized objective gives same cost (UL=2, LL=1) but with xe = 1 and yder = 1, x0 = 2

    # @constraints(model, begin
    #     # x0 >= xi
    #     x0 - yi + ye == d2
    #     yi - ye + yder == d1
    #     -xe + lambda + mu_e_up == 0 + mu_e_lo
    #     ci - lambda + mu_i_up == 0  + mu_i_lo
    #     cder - lambda + mu_der_up == 0  + mu_der_lo
    # end)
    # @constraint(model, [ye, yi] in MOI.SOS1([1.0, 2.0]))
    # @constraint(model, [ye, mu_e_up] in MOI.SOS1([1.0, 2.0]))
    # @constraint(model, [ye, mu_e_lo] in MOI.SOS1([1.0, 2.0]))
    # @constraint(model, [yi, mu_i_up] in MOI.SOS1([1.0, 2.0]))
    # @constraint(model, [yi, mu_i_lo] in MOI.SOS1([1.0, 2.0]))
    # @constraint(model, [yder, mu_der_up] in MOI.SOS1([1.0, 2.0]))
    # @constraint(model, [yder, mu_der_lo] in MOI.SOS1([1.0, 2.0]))
    # optimize!(model)

    # @expression(model, LLcost, cder * yder + ci * yi - xe * ye)
    # @test objective_value(model) ≈ 2.0
    # @test value(LLcost) ≈ 1.0
    # @test value(yder) ≈ 1
    # @test value(LLcost) ≈ 1.0
    # @test value(yi) ≈ 0.0
    # @test value(ye) ≈ 0.0
    # @test value(yder) ≈ 1.0
    # @test value(x0) ≈ 2.0

end
