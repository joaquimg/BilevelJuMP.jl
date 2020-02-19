using Revise
using JuMP,Gurobi,BilevelJuMP
includet("./bilevelJuMPExtension.jl")
# min -4sum(x) -3sum(y)
# s.t.
# y = argmin_y sum(y)
#      2x + y <= 4
#       x +2y <= 4
#       x     >= 0
#           y >= 0
#
# sol: x = 2, y = 0

# opt is the orginal problem
opt =  quote
    function some_model()
        blm = BilevelModel()
        sz = 10
        um = Upper(blm)
        lm = Lower(blm)
        @variable(um, x[1:sz])
        @variable(lm, y[1:sz])

        @objective(um, Min, -4sum(x) -3sum(y))

        @objective(lm, Min, sum(y))

        for i in 1:sz
            @constraint(lm, 2x[i]+y[i] <= 4)
            @constraint(lm, x[i]+2y[i] <= 4)
            @constraint(lm, x[i] >= 0)
            @constraint(lm, y[i] >= 0)
        end
        @constraint(lm,con[i in 1:sz],y[i]==1) #sign
        return blm
    end
end


# test.1 simple removal and addition of constarint named "con"
printstyled("****REMOVAL OF CONSTRAINT CON****\n";color=:blue)
opt = @delete_cons(opt,con)
mdl = eval(opt)()
optimize!(mdl,Gurobi.Optimizer())
@assert value(mdl[:y][1]) ≈ 0
printstyled("****ADDITION OF CONSTRAINT CON****\n";color=:blue)
opt = @add_lower_cons(opt,con,opt[:y][1] == 1)
mdl = eval(opt)()
optimize!(mdl,Gurobi.Optimizer())
@assert value(mdl[:y][1]) ≈ 1
printstyled("****SIMPLE TEST PASSED****\n";color=:green)


#test.2 complex removal and addition of constarint named "con" within loops
opt = [opt,opt]
n = 10
@time begin
for ii = 1:2
    for i = 1:2
        for j = 1:2
            printstyled("****REMOVAL OF CONSTRAINT CON****\n";color=:blue)
            opt[ii] = @delete_cons(opt[ii],con)
            f = eval(opt[ii])
            optimize!(f(),Gurobi.Optimizer(OutputFlag=0))
            sth = [0.5,0.3]
            idx = 2
            printstyled("****ADDITION OF CONSTRAINT CON****\n";color=:blue)
            opt[ii] = @add_lower_cons(opt[ii],con,sum(opt[ii][:y][i] + sth[idx]^2 for i in 1:n) == 10 + 0.1*(i+j))
            # println(opt[ii])
            f = eval(opt[ii])
            optimize!(f(),Gurobi.Optimizer(OutputFlag=0))
        end
    end
end
end
printstyled("****COMPLEX ADDITION TEST WITH LOOPS PASSED****\n";color=:green)

#test.3 batch addtion of constraints
@time begin
    for t in 1:2
        printstyled("****REMOVAL OF CONSTRAINT CON****\n";color=:blue)
        opt[t] = @delete_cons(opt[t],con)
        f = eval(opt[t])
        optimize!(f(),Gurobi.Optimizer(OutputFlag=0))
        sth = [1 for i in 1:n]
        printstyled("****ADDITION OF CONSTRAINT CON****\n";color=:blue)
        opt[t] = @add_lower_cons(opt[t],con[i = 1:n],opt[t][:y][i] == sth[i])
        # println(opt[t])
        f = eval(opt[t])
        optimize!(f(),Gurobi.Optimizer(OutputFlag=0))
    end
end
printstyled("****BATCH CONSTRAINTS ADDITION PASSED****\n";color=:green)
