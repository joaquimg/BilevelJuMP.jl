using BilevelJuMP
using JuMP
using Test

@testset "basic example (1)" begin
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

    new_model, lower_variables, lower_objective, lower_constraints , lower_sense = BilevelJuMP._build_single_model(model)

    
    #Check over number of variables
    @test length(lower_variables) == 1 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (2 + 1) # upper + lower

    #Check over number of constraints
    @test length(lower_constraints) == (4 + 0) #lower constraint + number of integer variables
    
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}()) == (2 + 3) # lessthan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.GreaterThan{Float64}}()) == (2 + 1) # lessthan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.SingleVariable, MOI.Integer}()) == (0 + 0) # number of constraints for integer represantation of the variables

    #Check over the objective function of the lower level
    x = lower_variables[1]
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}([MOI.ScalarAffineTerm(-1.0, x)],0.0)
    @test lower_sense == MOI.MIN_SENSE

end

@testset "basic example (2) - integer in lower level " begin
    model = BilevelModel()
    
    @variable(Upper(model), y)
    @variable(Upper(model), z)
    @variable(Lower(model), x, Int)

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

    new_model, lower_variables, lower_objective, lower_constraints, lower_sense = BilevelJuMP._build_single_model(model)
    

    #Check over number of variables
    @test length(lower_variables) == 1 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (2 + 1) # upper + lower

    #Check over number of constraints
    @test length(lower_constraints) == (4 + 1) #lower constraint + number of integer variables
    
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}()) == (2 + 3) # lessthan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.GreaterThan{Float64}}()) == (2 + 1) # lessthan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.SingleVariable, MOI.Integer}()) == (0 + 1) # number of constraints for integer represantation of the variables

    #Check over the objective function of the lower level
    x = lower_variables[1]
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}([MOI.ScalarAffineTerm(-1.0, x)],0.0)
    @test lower_sense == MOI.MIN_SENSE
end


@testset "basic example (3) - integer in upper level" begin
    model = BilevelModel()

    @variable(Upper(model), y)
    @variable(Upper(model), z, Int)
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

    new_model, lower_variables, lower_objective, lower_constraints, lower_sense = BilevelJuMP._build_single_model(model)

    #Check over number of variables
    @test length(lower_variables) == 1 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (2 + 1) # upper + lower

    #Check over number of constraints
    @test length(lower_constraints) == (4 + 0) #lower constraint + number of integer variables
    
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}()) == (2 + 3) # lessthan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.GreaterThan{Float64}}()) == (2 + 1) # lessthan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.SingleVariable, MOI.Integer}()) == (1 + 0) # number of constraints for integer represantation of the variables

    #Check over the objective function of the lower level
    x = lower_variables[1]
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}([MOI.ScalarAffineTerm(-1.0, x)],0.0)
    @test lower_sense == MOI.MIN_SENSE
end


@testset "basic example (4) - main" begin
    model = BilevelModel()

    I = 7 # maximum literals
    clauses = [[1,2,3],[-1,-4,3],[7,-6,4],[5,6,7]]

    # Upper level variables
    @variable(Upper(model), ya[i=1:I])          #7 variables
    @variable(Upper(model), yb[i=1:I])          #7 variables
    @variable(Upper(model), z)                  #1 variable
    #---------------------------------------------------------
    #   15 variables
    #---------------------------------------------------------

    #Lower level variables
    @variable(Lower(model), x[i=1:I])           #7 variables
    #---------------------------------------------------------
    #   7 variables
    #---------------------------------------------------------

    # Upper level objecive function
    @objective(Upper(model), Min, sum(x[i] for i in 1:I) - z)

    # Upper level constraints
    @constraint(Upper(model), ca, z <= 1)                       #1 LessThan
    @constraint(Upper(model), cb, z >= 0)                       #1 GreaterThan
    @constraint(Upper(model), c1[i=1:I], ya[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c2[i=1:I], ya[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c3[i=1:I], yb[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c4[i=1:I], yb[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c5[i=1:I], ya[i] + yb[i] == 1)    #7 Equlity
    @constraint(Upper(model), cc[k in eachindex(clauses)], sum(i > 0 ? ya[i] : yb[-i] for i in clauses[k]) >= z) #4 GreaterThan
    #---------------------------------------------------------
    #   19  GreaterThan
    #   15  LessThan
    #   7   Equlity
    #   0   Integer
    #   31  Total
    #---------------------------------------------------------

    # Followed by the objective and constraints of the lower problem:

    # Lower objective function
    @objective(Lower(model), Max, sum(x[i] for i in 1:I))

    # Lower constraints
    @constraint(Lower(model), b1[i=1:I], x[i] >= 0)         #7  GreaterThan  
    @constraint(Lower(model), b2[i=1:I], x[i] <= ya[i])     #7  LessThan
    @constraint(Lower(model), b3[i=1:I], x[i] <= yb[i])     #7  LessThan
    #---------------------------------------------------------
    #   7   GreaterThan
    #   14  LessThan
    #   0   Equlity
    #   0   Integer
    #   21  Total
    #---------------------------------------------------------

    new_model, lower_variables, lower_objective, lower_constraints, lower_sense = BilevelJuMP._build_single_model(model)

    #Check over number of variables
    @test length(lower_variables) == 7 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (15 + 7) # upper + lower

    #Check over number of constraints
    @test length(lower_constraints) == (21 + 0) #lower constraint + number of integer variables
    
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}()) == (15 + 14) # GreaterThan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.GreaterThan{Float64}}()) == (19 + 7) # LessThan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.EqualTo{Float64}}()) == (7 + 0) # EqualTo
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.SingleVariable, MOI.Integer}()) == (0 + 0) # number of constraints for integer represantation of the variables

    # Chenking the objective function of the lower problem
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}([MOI.ScalarAffineTerm(1.0, x) for x in lower_variables],0.0)
    @test lower_sense == MOI.MAX_SENSE
    
end



@testset "basic example (5) - integer in lower level" begin
    model = BilevelModel()

    I = 7 # maximum literals
    clauses = [[1,2,3],[-1,-4,3],[7,-6,4],[5,6,7]]

    # Upper level variables
    @variable(Upper(model), ya[i=1:I])          #7 variables
    @variable(Upper(model), yb[i=1:I])          #7 variables
    @variable(Upper(model), z)                  #1 variable
    #---------------------------------------------------------
    #   15 variables
    #---------------------------------------------------------

    #Lower level variables
    @variable(Lower(model), x[i=1:I], Int)           #7 variables
    #---------------------------------------------------------
    #   7 variables
    #---------------------------------------------------------

    # Upper level objecive function
    @objective(Upper(model), Min, sum(x[i] for i in 1:I) - z)

    # Upper level constraints
    @constraint(Upper(model), ca, z <= 1)                       #1 LessThan
    @constraint(Upper(model), cb, z >= 0)                       #1 GreaterThan
    @constraint(Upper(model), c1[i=1:I], ya[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c2[i=1:I], ya[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c3[i=1:I], yb[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c4[i=1:I], yb[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c5[i=1:I], ya[i] + yb[i] == 1)    #7 Equlity
    @constraint(Upper(model), cc[k in eachindex(clauses)], sum(i > 0 ? ya[i] : yb[-i] for i in clauses[k]) >= z) #4 GreaterThan
    #---------------------------------------------------------
    #   19  GreaterThan
    #   15  LessThan
    #   7   Equlity
    #   0   Integer
    #   31  Total
    #---------------------------------------------------------

    # Followed by the objective and constraints of the lower problem:

    # Lower objective function
    @objective(Lower(model), Max, sum(x[i] for i in 1:I))

    # Lower constraints
    @constraint(Lower(model), b1[i=1:I], x[i] >= 0)         #7  GreaterThan  
    @constraint(Lower(model), b2[i=1:I], x[i] <= ya[i])     #7  LessThan
    @constraint(Lower(model), b3[i=1:I], x[i] <= yb[i])     #7  LessThan
    #---------------------------------------------------------
    #   7   GreaterThan
    #   14  LessThan
    #   0   Equlity
    #   7   Integer
    #   21  Total
    #---------------------------------------------------------

    new_model, lower_variables, lower_objective, lower_constraints, lower_sense = BilevelJuMP._build_single_model(model)

    #Check over number of variables
    @test length(lower_variables) == 7 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (15 + 7) # upper + lower

    #Check over number of constraints
    @test length(lower_constraints) == (21 + 7) #lower constraint + number of integer variables
    
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}()) == (15 + 14) # GreaterThan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.GreaterThan{Float64}}()) == (19 + 7) # LessThan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.EqualTo{Float64}}()) == (7 + 0) # EqualTo
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.SingleVariable, MOI.Integer}()) == (0 + 7) # number of constraints for integer represantation of the variables

    # Chenking the objective function of the lower problem
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}([MOI.ScalarAffineTerm(1.0, x) for x in lower_variables],0.0)
    @test lower_sense == MOI.MAX_SENSE
end


@testset "basic example (6) - integer in lower level" begin
    model = BilevelModel()

    I = 7 # maximum literals
    clauses = [[1,2,3],[-1,-4,3],[7,-6,4],[5,6,7]]

    # Upper level variables
    @variable(Upper(model), ya[i=1:I])                          #7 variables
    @variable(Upper(model), yb[i=1:I], Int)                     #7 variables
    @variable(Upper(model), z, Int)                             #1 variable
    #---------------------------------------------------------
    #   15 variables
    #---------------------------------------------------------

    #Lower level variables
    @variable(Lower(model), x[i=1:I])                   #7 variables
    #---------------------------------------------------------
    #   7 variables
    #---------------------------------------------------------

    # Upper level objecive function
    @objective(Upper(model), Min, sum(x[i] for i in 1:I) - z)

    # Upper level constraints
    @constraint(Upper(model), ca, z <= 1)                       #1 LessThan
    @constraint(Upper(model), cb, z >= 0)                       #1 GreaterThan
    @constraint(Upper(model), c1[i=1:I], ya[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c2[i=1:I], ya[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c3[i=1:I], yb[i] >= 0)            #7 GreaterThan
    @constraint(Upper(model), c4[i=1:I], yb[i] <= 1)            #7 LessThan
    @constraint(Upper(model), c5[i=1:I], ya[i] + yb[i] == 1)    #7 Equlity
    @constraint(Upper(model), cc[k in eachindex(clauses)], sum(i > 0 ? ya[i] : yb[-i] for i in clauses[k]) >= z) #4 GreaterThan
    #---------------------------------------------------------
    #   19  GreaterThan
    #   15  LessThan
    #   7   Equlity
    #   8   Integer
    #   31  Total
    #---------------------------------------------------------

    # Followed by the objective and constraints of the lower problem:

    # Lower objective function
    @objective(Lower(model), Max, sum(x[i] for i in 1:I))

    # Lower constraints
    @constraint(Lower(model), b1[i=1:I], x[i] >= 0)         #7  GreaterThan  
    @constraint(Lower(model), b2[i=1:I], x[i] <= ya[i])     #7  LessThan
    @constraint(Lower(model), b3[i=1:I], x[i] <= yb[i])     #7  LessThan
    #---------------------------------------------------------
    #   7   GreaterThan
    #   14  LessThan
    #   0   Equlity
    #   0   Integer
    #   21  Total
    #---------------------------------------------------------

    new_model, lower_variables, lower_objective, lower_constraints, lower_sense = BilevelJuMP._build_single_model(model)

    #Check over number of variables
    @test length(lower_variables) == 7 # lower
    @test MOI.get(new_model, MOI.NumberOfVariables()) == (15 + 7) # upper + lower

    #Check over number of constraints
    @test length(lower_constraints) == (21 + 0) #lower constraint + number of integer variables
    
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}()) == (15 + 14) # GreaterThan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.GreaterThan{Float64}}()) == (19 + 7) # LessThan
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.EqualTo{Float64}}()) == (7 + 0) # EqualTo
    @test MOI.get(new_model, MOI.NumberOfConstraints{MOI.SingleVariable, MOI.Integer}()) == (8 + 0) # number of constraints for integer represantation of the variables

    # Chenking the objective function of the lower problem
    @test lower_objective ≈ MOI.ScalarAffineFunction{Float64}([MOI.ScalarAffineTerm(1.0, x) for x in lower_variables],0.0)
    @test lower_sense == MOI.MAX_SENSE
end
