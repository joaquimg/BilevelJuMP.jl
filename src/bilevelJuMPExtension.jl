macro delete_cons(model,cref)
    model_string = string(model)
    cref_string = string(cref)
    s = quote
        local con = $cref_string
        # println(con)
        local model_local = Meta.parse($model_string)
        local semi_locals = Base.@locals # get the local vars within the loop
        local model_fixed = remove_local_var(model_local,semi_locals)
        local s = repr(eval(model_fixed))
        local p = Regex("(?<=\\n)(.*\\bconstraint\\b.*\\b$con\\b.*)(?=\\n)")
        # p = Regex("$con")
        # println(p)
        local snew = replace(s,p => "")
        local model_new = Meta.parse(snew)#默认会套上一层quote "x" -> "quote x end"
        eval(model_new)
    end
    return s
end
macro add_upper_cons(model,expression...)#Almost the same as add_lower_cons except for Upper()/Lower()
    model_string = string(model)
    expression_string = [string(e) for e in expression]
    return quote
    local model_local = Meta.parse($model_string)
    local expression_local = [Meta.parse(x) for x in $expression_string]
    @assert length($expression_string) <= 2
    @assert length($expression_string) >= 1
    local semi_locals = Base.@locals # get the local vars within the loop
    local model_fixed = remove_local_var(model_local,semi_locals)
    local s = repr(eval(model_fixed))
    local p = r"\n.*return"
    local blm = match(r"\b.*(?==.*Bilevel)",s).match
    local vars_temp = [split(x.match," ")[3] for x in eachmatch(r"(?<=@variable).*",s)]
    # println(vars_temp)
    local vars = [Meta.parse(replace(x,r"(?<=\[).*(?=\])|\]|\["=>"")) for x in vars_temp] #get the vars defined in the model
    # println(vars)
    if length(expression_local) == 1
        local locals = []
    else
        local cref,locals = remain_cref(expression_local[1])
    end
    local ctr, = remain_opt_var(model_local,expression_local[end],vars,semi_locals,locals)#modify the contraint expression, always the last argument
    ctr = string(ctr)
    cref = string(cref)
    if length(expression_local) == 1
        local constraint = "@constraint(Upper($blm),$(ctr))"
    elseif length(expression_local) == 2
        local constraint = "@constraint(Upper($blm),$(cref),$(ctr))"
    end
    local m = replace(s,p=>(pstr->"$constraint\n"*pstr))
    local model_new = Meta.parse(m)
    # @info("constraint ***$constraint*** added")
    eval(model_new)
    # println(m)
    end
end
macro add_lower_cons(model,expression...)
    model_string = string(model)
    expression_string = [string(e) for e in expression]
    return quote
    local model_local = Meta.parse($model_string)
    local expression_local = [Meta.parse(x) for x in $expression_string]
    @assert length($expression_string) <= 2
    @assert length($expression_string) >= 1
    local semi_locals = Base.@locals # get the local vars within the loop
    local model_fixed = remove_local_var(model_local,semi_locals)
    local s = repr(eval(model_fixed))
    local p = r"\n.*return"
    local blm = match(r"\b.*(?==.*Bilevel)",s).match
    local vars_temp = [split(x.match," ")[3] for x in eachmatch(r"(?<=@variable).*",s)]
    # println(vars_temp)
    local vars = [Meta.parse(replace(x,r"(?<=\[).*(?=\])|\]|\["=>"")) for x in vars_temp] #get the vars defined in the model
    # println(vars)
    if length(expression_local) == 1
        local locals = []
    else
        local cref,locals = remain_cref(expression_local[1])
    end
    local ctr, = remain_opt_var(model_local,expression_local[end],vars,semi_locals,locals)#modify the contraint expression, always the last argument
    ctr = string(ctr)
    cref = string(cref)
    if length(expression_local) == 1
        local constraint = "@constraint(Lower($blm),$(ctr))"
    elseif length(expression_local) == 2
        local constraint = "@constraint(Lower($blm),$(cref),$(ctr))"
    end
    local m = replace(s,p=>(pstr->"$constraint\n"*pstr))
    local model_new = Meta.parse(m)
    # @info("constraint ***$constraint*** added")
    eval(model_new)
    # println(m)
    end
end
function remain_opt_var(model,expression,varlist,semi_locals,local_var=[])
    # println(expression)
    # println(model)
    lv = [x for x in local_var]
    is_numeric = true
    if expression.head == :generator
        append!(lv,[x.args[1] for x in expression.args if x.head == :(=)])
        append!(lv,[x.args[2].args[1] for x in expression.args if x.head == :filter])
    end
    if expression.head == :ref && expression.args[1] == model
        println("success")
        popfirst!(expression.args)#remove model name e.g mdl[:x] -> :(:x)
        var = expression.args[1].value#remove quote :(:x) -> :x
        @assert var in varlist
        expression = var
        return expression,false
    end
    for (k,v) in enumerate(expression.args)
        if typeof(v) == Expr
            expression.args[k],is_numeric_temp = remain_opt_var(model,v,varlist,semi_locals,lv)
        else#Symbol
            if v in lv
                is_numeric_temp = false
            elseif v in keys(semi_locals)
                expression.args[k] = semi_locals[v]
                is_numeric_temp = true
            else
                expression.args[k] = @eval Main $v
                is_numeric_temp = true
            end
        end
        is_numeric = is_numeric && is_numeric_temp
    end
    if is_numeric
        # println(expression)
        expression = @eval Main $expression
    end
    return expression,is_numeric
end
function remove_local_var(m::Expr,locals::Dict)
    model = deepcopy(m)
    @assert model.head == :ref
    if model.args[2] in keys(locals)
        model.args[2] = locals[model.args[2]]
    else
        v = model.args[2]
        model.args[2] = @eval Main $v
    end
    model.args[1] = remove_local_var(model.args[1],locals)
    return model
end
function remove_local_var(m::Symbol,locals::Dict)
    return m
end
function remain_cref(expression)
    local_var = []
    if typeof(expression) == Symbol
        return expression,local_var
    else
        nothing
    end
    @assert expression.head == :ref
    for k in 2:length(expression.args)
        # println(dump(expression.args[k]))
        @assert expression.args[k].head == :kw || expression.args[k].args[1] == :in
        if expression.args[k].head == :kw
            v = expression.args[k].args[2]
            expression.args[k].args[2] = @eval Main $v
            push!(local_var,expression.args[k].args[1])
        else
            v = expression.args[k].args[3]
            expression.args[k].args[3] = @eval Main $v
            push!(local_var,expression.args[k].args[2])
        end
    end
    return expression,local_var
end
