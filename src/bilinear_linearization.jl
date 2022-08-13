#=
Methods in this file are for supporting the linearization of bilinear products of lower dual and 
lower primal variables in the upper level problem. 
Details can be found in https://ieeexplore.ieee.org/abstract/document/9729553
=#
using Dates
PushVectors.finish!(v::AbstractVector) = v

"""
    non_zero_idxs_except_one(v::AbstractVector, idx::Int)

Return indices of all the non-zero values in v except idx
"""
function non_zero_idxs_except_one(v::AbstractVector, idx::Int)
    idxs = findall(!iszero, v)
    if !isempty(idxs)
        deleteat!(idxs, findfirst(x->x==idx, idxs))  # findfirst can return nothing
    end
    return idxs
end


# switch input of I,J for finding cs
function non_zero_idxs_except_one_IJV(I, J, col::Int, idx::Int)
    rs = PushVector{Int}()
    for (i,v) in enumerate(J)
        if v == col && I[i] != idx
            push!(rs, I[i])
        end
    end
    return rs  # need finish! ?
end


struct UnderDeterminedException <: Exception end


"""
    recursive_col_search(A::AbstractArray, row::Int, col::Int, rows::AbstractVector{Int}, cols::AbstractVector{Int})

Given a row, col in an array A find all the connected rows and cols starting with searching col, 
where "connected" rows and cols are those with non-zero values.
Search is stopped if any redundant rows or cols are added to the output arrays 
(which indicates that there is a loop in the connections and the system of equations is underdetermined).

Returns Vector{Int}, Vector{Int}, Bool where the Bool indicates if redundanct rows or columns were found
(true indicates underdertermined system).
"""
function recursive_col_search(A::AbstractArray, row::Int, col::Int, 
    rows::AbstractVector{Int}, cols::AbstractVector{Int})
    rs = non_zero_idxs_except_one(A[:, col], row)
    if any(r in rows for r in rs)
        rr = intersect(rs, rows)
        @debug("Returning early from recursive_col_search due to redundant row(s)! ($rr)")
        throw(UnderDeterminedException())
    end
    push!(rows, rs...)
    for r in rs
        cs = non_zero_idxs_except_one(A[r, :], col)
        if any(c in cols for c in cs)
            cc = intersect(cs, cols)
            @debug("Returning early from recursive_col_search due to redundant column(s)! ($cc)")
            throw(UnderDeterminedException())
        end
        push!(cols, cs...)
        for c in cs
            recursive_col_search(A, r, c, rows, cols)
        end
    end
    return finish!(rows), finish!(cols)
end

# a version of recursive_col_search that works with I,J,vals = findnz(V)
function recursive_col_search_IJV(I::Vector{Int}, J::Vector{Int}, vals::Vector{<:Real}, row::Int, col::Int, 
    rows::AbstractVector{Int}, cols::AbstractVector{Int})

    rs = non_zero_idxs_except_one_IJV(I, J, col, row)
    # rs = non_zero_idxs_except_one(A[:, col], row)
    if any(r in rows for r in rs)
        rr = intersect(rs, rows)
        @debug("Returning early from recursive_col_search due to redundant row(s)! ($rr)")
        throw(UnderDeterminedException())
    end
    push!(rows, rs...)
    for r in rs
        cs = non_zero_idxs_except_one_IJV(J, I, r, col)
        # cs = non_zero_idxs_except_one(A[r, :], col)
        if any(c in cols for c in cs)
            cc = intersect(cs, cols)
            @debug("Returning early from recursive_col_search due to redundant column(s)! ($cc)")
            throw(UnderDeterminedException())
        end
        push!(cols, cs...)
        for c in cs
            recursive_col_search_IJV(I, J, vals, r, c, rows, cols)
        end
    end
    return finish!(rows), finish!(cols)
end


"""
    find_connected_rows_cols(A::AbstractArray, row::Int, col::Int; skip_1st_col_check=false)
    
Given a row, col location to start from in an array, find all the connected rows and columns
where connected is defined by following non-zero values in the array, starting with all non-zero
values in the given row (except the starting row, col)

Return Int, Int for rows to add primal equations, and columns to add dual equations

A = [1 0 0 1 0 0; 
     0 0 1 1 0 0;
     0 0 1 0 1 0;
     0 1 1 0 0 0;
     1 0 0 0 0 1]

find_connected_rows_cols(A, 1, 1)
([1, 2, 3, 4], [4, 3, 5, 2])

Example 5.2 in paper:
using BlockArrays
V = BlockArray{Float64}(undef_blocks, [3,3], [3,3,3,1,3])
setblock!(V, Matrix{Float64}(-I, 3, 3), 1, 1)
setblock!(V, Matrix{Float64}( I, 3, 3), 1, 2)
setblock!(V, Matrix{Float64}( I, 3, 3), 1, 3)
setblock!(V, Matrix{Float64}( I, 3, 3), 2, 3)
setblock!(V, Matrix{Float64}( I, 3, 3), 2, 5)
setblock!(V, zeros(3,3), 2, 1)
setblock!(V, zeros(3,3), 2, 2)
setblock!(V, zeros(3,3), 1, 5)
setblock!(V, zeros(3,1), 1, 4)
setblock!(V, ones(3,1), 2, 4)

find_connected_rows_cols(V, 1, 1; skip_1st_col_check=true)
([1, 4, 5, 6, 2, 3], [4, 7, 10, 11, 8, 12, 2, 5, 9, 13, 3, 6])
"""
function find_connected_rows_cols(A::AbstractArray, row::Int, col::Int; 
    skip_1st_col_check=false,
    finding_blocks=false,
    )
    if A[row, col] == 0 
        throw(@error("Linearization is undefined when the dual variable is not associated with the primal variable."))
    end
    redundant_vals = false
    # step 1 check if all non-zeros in A[:, col], if so the dual constraint gives linearization
    if !skip_1st_col_check && length(findall(!iszero, A[:, col])) == 1
        return [], [col], redundant_vals
    end
    # step 2 add 1st row and any other non-zero columns
    rows = PushVector{Int}()
    push!(rows, row)
    if finding_blocks
        cols_to_check = findall(!iszero, A[row, :])
    else
        cols_to_check = non_zero_idxs_except_one(A[row, :], col)
    end
    cols = PushVector{Int}()
    push!(cols, cols_to_check...)
    # step 3 recursive search to find all connections
    rows_to_add, cols_to_add = Int[], Int[]
    for c in cols_to_check
        try
            rows_to_add, cols_to_add = recursive_col_search(A, row, c, PushVector{Int}(), PushVector{Int}())
        catch e
            if isa(e, UnderDeterminedException)
                if finding_blocks  # then we still need to add the connected rows and cols
                    for r in non_zero_idxs_except_one(A[:, c], row)
                        push!(rows, r)
                        push!(cols, findall(!iszero, A[r, :])...)
                    end
                    rows = unique(rows)  # unique! does not work with PushVector
                    cols = unique(cols)
                else
                    redundant_vals = true
                end
                break
            else rethrow(e)
            end
        end
        push!(rows, rows_to_add...)
        push!(cols, cols_to_add...)
    end
    
    return finish!(rows), finish!(cols), redundant_vals
end


function find_connected_rows_cols_cached(A, I, J, vals, row::Int, col::Int; 
    skip_1st_col_check=false,
    finding_blocks=false,
    )
    
    if (row, col, skip_1st_col_check, finding_blocks) ∉ keys(cache)
        if A[row, col] == 0 
            throw(@error("Linearization is undefined when the dual variable is not associated with the primal variable."))
        end
        redundant_vals = false
        # step 1 check if all non-zeros in A[:, col], if so the dual constraint gives linearization
        if !skip_1st_col_check && length(findall(!iszero, A[:, col])) == 1  # TODO an IJV method for this?
            return [], [col], redundant_vals
        end
        # step 2 add 1st row and any other non-zero columns
        rows = Int[]
        push!(rows, row)
        if finding_blocks
            cols_to_check = findall(!iszero, A[row, :])  # TODO an IJV method for this?
        else
            cols_to_check = non_zero_idxs_except_one_IJV(J, I, row, col) #non_zero_idxs_except_one(A[row, :], col)
        end
        cols = Int[]
        push!(cols, cols_to_check...)
        # step 3 recursive search to find all connections
        rows_to_add, cols_to_add = Int[], Int[]
        for c in cols_to_check
            try
                rows_to_add, cols_to_add = recursive_col_search_IJV(I,J,vals, row, c, Int[], Int[])
            catch e
                if isa(e, UnderDeterminedException)
                    if finding_blocks  # then we still need to add the connected rows and cols
                        for r in non_zero_idxs_except_one_IJV(I, J, c, row) #non_zero_idxs_except_one(A[:, c], row)
                            push!(rows, r)
                            push!(cols, findall(!iszero, A[r, :])...)  # TODO an IJV method for this?
                        end
                        rows = unique(rows)  # unique! does not work with PushVector
                        cols = unique(cols)
                    else
                        redundant_vals = true
                    end
                    break
                else rethrow(e)
                end
            end
            push!(rows, rows_to_add...)
            push!(cols, cols_to_add...)
        end
        
        cache[(row, col, skip_1st_col_check, finding_blocks)] = finish!(rows), finish!(cols), redundant_vals
    end
    return cache[(row, col, skip_1st_col_check, finding_blocks)]
end


function get_coef(var::MOI.VariableIndex, safts::Vector{MOI.ScalarAffineTerm{R}}) where R <: Real
    coef = 0
    for saft in safts
        if var == saft.variable
            coef = saft.coefficient
        end
    end
    return coef
end


function get_coef(var1::MOI.VariableIndex, var2::MOI.VariableIndex, sqts::Vector{MOI.ScalarQuadraticTerm{R}}) where R <: Real
    coef = 0
    for sqft in sqts
        if var1 == sqft.variable_1 && var2 == sqft.variable_2
            coef = sqft.coefficient
        end
    end
    return coef
end


function get_coef_matrix_and_rhs_vec(m, 
	constraint_indices::Array{
		MOI.ConstraintIndex{
			MOI.ScalarAffineFunction{Float64}, 
			MOI.GreaterThan{Float64}
		}, 1}
	)
	nrows = length(constraint_indices)
	E = spzeros(nrows, MOI.get(m, MOI.NumberOfVariables()))
	f = -Inf*ones(nrows)

	for (r, ci) in enumerate(constraint_indices)
		con_func = MOI.get(m, MOI.ConstraintFunction(), ci)
		for term in con_func.terms
			E[r, term.variable.value] = term.coefficient
		end
		f[r] = MOI.get(m, MOI.ConstraintSet(), ci).lower
	end
	return E, f
end


# can replace the following SingleVariable limits with an MOI function for getting bounds?
function get_coef_matrix_and_rhs_vec(m, 
	constraint_indices::Array{
		MOI.ConstraintIndex{
			MOI.VariableIndex, 
			MOI.GreaterThan{Float64}
		}, 1}
	)
	f = -Inf*ones(MOI.get(m, MOI.NumberOfVariables()))

	for ci in constraint_indices
		var_index = MOI.get(m, MOI.ConstraintFunction(), ci)
		f[var_index.value] = MOI.get(m, MOI.ConstraintSet(), ci).lower
	end
	return f
end


function get_coef_matrix_and_rhs_vec(m, 
	constraint_indices::Array{MOI.ConstraintIndex{
			MOI.VariableIndex, 
			MOI.LessThan{Float64}},
		1}
	)
	d = Inf*ones(MOI.get(m, MOI.NumberOfVariables()))

	for ci in constraint_indices
		var_index = MOI.get(m, MOI.ConstraintFunction(), ci)
		d[var_index.value] = MOI.get(m, MOI.ConstraintSet(), ci).upper
	end
	return d
end


function get_coef_matrix_and_rhs_vec(m, 
	constraint_indices::Array{
		MOI.ConstraintIndex{
			MOI.ScalarAffineFunction{Float64}, 
			MOI.LessThan{Float64}},
		1}
	)
	nrows = length(constraint_indices)
	C = spzeros(nrows, MOI.get(m, MOI.NumberOfVariables()))
	d = Inf*ones(nrows)
	for (r, ci) in enumerate(constraint_indices)
		con_func = MOI.get(m, MOI.ConstraintFunction(), ci)
		for term in con_func.terms
			C[r, term.variable.value] = term.coefficient
		end
		d[r] = MOI.get(m, MOI.ConstraintSet(), ci).upper
	end
	return C, d
end


function get_coef_matrix_and_rhs_vec(m, 
    constraint_indices::Array{
        MOI.ConstraintIndex{
            MOI.ScalarAffineFunction{Float64}, 
            MOI.EqualTo{Float64}},
        1}
    )
    nrows = length(constraint_indices)
    V = spzeros(nrows, MOI.get(m, MOI.NumberOfVariables()))
    w = spzeros(nrows)
    for (r, ci) in enumerate(constraint_indices)
        con_func = MOI.get(m, MOI.ConstraintFunction(), ci)
        con_set = MOI.get(m, MOI.ConstraintSet(), ci)
        for term in con_func.terms
            V[r, term.variable.value] = term.coefficient
        end
        w[r] = con_set.value
    end
    return V, w
end


"""
    standard_form(m; upper_var_indices=Vector{MOI.VariableIndex}())

Given:
    
    A[x:y] = b, C[x;y] ≤ d, E[x;y] ≥ f

collect all of y bounds into:

    yl ≤ y ≤ yu

and put all inequality constraints into equalities with slack variables s:

    Ux + V [y; s] = [b; d; f]

NOTE: U and V are sparse arrays with columns for all variables in model s.t. that variable indices line up
"""
function standard_form(m; upper_var_indices=Vector{MOI.VariableIndex}())
	nvars = MOI.get(m, MOI.NumberOfVariables())
	con_types = MOI.get(m, MOI.ListOfConstraintTypesPresent())

    n_equality_cons = 0  # A[x;y] = b

    @info """starting A,b at $(Dates.format(now(), "HH:MM:SS"))"""
	if (MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64}) in con_types

		eq_con_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.ScalarAffineFunction{Float64}, 
			MOI.EqualTo{Float64}
		}());

		A, b = get_coef_matrix_and_rhs_vec(m, eq_con_indices)
        n_equality_cons = size(A,1)
	else
		A, b = spzeros(0, nvars), spzeros(0)
	end
    # U = part of A for upper_var_indices
    # V = rest of A

    #=
    For each ScalarAffineFunction ≥ or ≤ Float64 we add a slack variable and make the constraint
    an EqualTo{Float64}
    =#
    n_lessthan_cons = 0  # C[x;y] ≤ d

    @info """starting C,d at $(Dates.format(now(), "HH:MM:SS"))"""
	if (MOI.ScalarAffineFunction{Float64}, MOI.LessThan{Float64}) in con_types

		lt_con_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.ScalarAffineFunction{Float64}, 
			MOI.LessThan{Float64}
		}());

		C, d = BilevelJuMP.get_coef_matrix_and_rhs_vec(m, lt_con_indices)
	else
		C, d = spzeros(0, nvars), spzeros(0)
	end
	
    n_greaterthan_cons = 0  # E[x;y] ≥ f

    @info """starting E,t at $(Dates.format(now(), "HH:MM:SS"))"""
	if (MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64}) in con_types

		gt_con_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.ScalarAffineFunction{Float64}, 
			MOI.GreaterThan{Float64}
		}());

		E, f = BilevelJuMP.get_coef_matrix_and_rhs_vec(m, gt_con_indices)
	else
		E, f = spzeros(0, nvars), spzeros(0)
	end
	
    #=
    For each SingleVariable GreaterThan{Float64} or LessThan{Float64} we fill in the bounds
    yl and yu
    =#

    @info """starting yl at $(Dates.format(now(), "HH:MM:SS"))"""
    yl = -Inf*ones(MOI.get(m, MOI.NumberOfVariables()))
	if (MOI.VariableIndex, MOI.GreaterThan{Float64}) in con_types

		singleVar_gt_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.VariableIndex, 
			MOI.GreaterThan{Float64}
		}());

		yl = BilevelJuMP.get_coef_matrix_and_rhs_vec(m, singleVar_gt_indices)
	end
	
    @info """starting yu at $(Dates.format(now(), "HH:MM:SS"))"""
    yu = Inf*ones(MOI.get(m, MOI.NumberOfVariables()))
	if (MOI.VariableIndex, MOI.LessThan{Float64}) in con_types

		singleVar_lt_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.VariableIndex, 
			MOI.LessThan{Float64}
		}());

		yu = get_coef_matrix_and_rhs_vec(m, singleVar_lt_indices)
	end

    # remove rows from C that only apply to one variable by moving them to the bounds in yu

    @info """starting remove rows from C at $(Dates.format(now(), "HH:MM:SS"))"""
    rows_to_remove = Int[]
    for r in 1:size(C,1)
        if length(findall(!iszero, C[r, :])) == 1  # only one non-zero value in row
            # then we remove the row and move it to the bounds (if the RHS is lower/higher than the upper/lower bound)
            push!(rows_to_remove, r)
            ci = findall(!iszero, C[r, :])[1]
            upbound = d[r] / C[r,ci]   # C [x;y] ≤ d
            if upbound < yu[ci]
                yu[ci] = upbound
            end
        end
    end
    C = C[setdiff(1:end, rows_to_remove),:]
    d = d[setdiff(1:end, rows_to_remove)]
    n_lessthan_cons = size(C,1)

    # remove rows from E that only apply to one variable by moving them to the bounds in yl

    @info """starting remove rows from E at $(Dates.format(now(), "HH:MM:SS"))"""
    rows_to_remove = Int[]
    for r in 1:size(E,1)
        if length(findall(!iszero, E[r, :])) == 1  # only one non-zero value in row
            # then we remove the row and move it to the bounds (if the RHS is lower/higher than the upper/lower bound)
            push!(rows_to_remove, r)
            ci = findall(!iszero, E[r, :])[1]
            lobound = f[r] / E[r,ci]
            if lobound > yl[ci]
                yl[ci] = lobound
            end
        end
    end
    E = E[setdiff(1:end, rows_to_remove),:]
    f = f[setdiff(1:end, rows_to_remove)]
    n_greaterthan_cons = size(E,1)


    #=
    Build V = | A       |
              | C  I    |
              | E     I |
    =#

    @info """starting build V at $(Dates.format(now(), "HH:MM:SS"))"""
    n_vars = size(A,2)
    n_rows_V = n_equality_cons + n_greaterthan_cons + n_lessthan_cons
    n_cols_V = n_vars + n_greaterthan_cons + n_lessthan_cons
    V = spzeros(n_rows_V, n_cols_V)
    V[1:n_equality_cons, 1:n_vars] = A
    GC.gc()

    V[n_equality_cons+1 : n_equality_cons+n_lessthan_cons, 1 : n_vars] = C
    GC.gc()
    V[n_equality_cons+1 : n_equality_cons+n_lessthan_cons, n_vars+1 : n_vars+n_lessthan_cons] = Matrix(I, n_lessthan_cons, n_lessthan_cons)
    GC.gc()

    V[n_equality_cons+n_lessthan_cons+1 : n_equality_cons+n_greaterthan_cons+n_lessthan_cons, 1:n_vars] = E
    GC.gc()
    V[n_equality_cons+n_lessthan_cons+1 : n_equality_cons+n_greaterthan_cons+n_lessthan_cons, 
      n_vars+n_lessthan_cons+1 : n_vars+n_greaterthan_cons+n_lessthan_cons] = Matrix(-I, n_greaterthan_cons, n_greaterthan_cons)
    GC.gc()

    # zero out the columns in V for upper level variables and build U

    @info """starting build U,V loop at $(Dates.format(now(), "HH:MM:SS"))"""
    rows, cols, vals = findnz(V)
    Urows = PushVector{Int}()
    Ucols = PushVector{Int}()
    Uvals = PushVector{Float64}()

    for col in upper_var_indices
        indices_to_mv = findall(i->i==col.value, cols)
        push!(Urows, [rows[i] for i in indices_to_mv]...)
        push!(Ucols, [cols[i] for i in indices_to_mv]...)
        push!(Uvals, [vals[i] for i in indices_to_mv]...)
        deleteat!(rows, indices_to_mv)
        deleteat!(cols, indices_to_mv)
        deleteat!(vals, indices_to_mv)
    end
    GC.gc()

    V = sparse(rows, cols, vals, n_rows_V, n_cols_V)
    U = sparse(finish!(Urows), finish!(Ucols), finish!(Uvals), n_rows_V, n_cols_V)
    # every time a new V is created for a problem we need to empty the cache used in find_connected_rows_cols_cached
    empty!(cache)
    w = [b; d; f]
    @info """done standard_form at $(Dates.format(now(), "HH:MM:SS"))"""

    return U, V, w # , yu, yl, n_equality_cons, C, E
    # TODO use n_equality_cons to check rows from find_connected_rows_cols for values corresponding to constraints with slack variables
end


"""
    check_upper_objective_for_bilinear_linearization(upper, upper_to_lower_var_indices, upper_var_to_lower_ctr)

Construct the set A_N, which is the indices of lower level variables in the upper level 
objective of the form λ^T A y, where λ are the dual variables of lower level equality constraints and
y are lower level primal variables.

Also find the quadratic upper objective terms with lower primal * lower dual and return the maps for
the upper variable index (of the lower dual variable):
- to the lower primal term and 
- to the quadratic term in the UL objective

"""
function check_upper_objective_for_bilinear_linearization(upper, upper_to_lower_var_indices, upper_var_to_lower_ctr)
    nt = Threads.nthreads()

    A_N = Vector{PushVector{Int}}()
    upper_dual_to_quad_term = Vector{Dict{BilevelJuMP.VI, Dict{BilevelJuMP.VI, Float64}}}()
    upper_dual_to_lower_primal = Vector{Dict{BilevelJuMP.VI, Vector{BilevelJuMP.VI}}}()
    lower_primal_var_to_lower_con = Vector{Dict{BilevelJuMP.VI, BilevelJuMP.CI}}()

    for _ in 1:nt
        push!(A_N, PushVector{Int}())
        push!(upper_dual_to_quad_term, Dict{BilevelJuMP.VI, Dict{BilevelJuMP.VI, Float64}}())
        push!(upper_dual_to_lower_primal, Dict{BilevelJuMP.VI, Vector{BilevelJuMP.VI}}())
        push!(lower_primal_var_to_lower_con, Dict{BilevelJuMP.VI, BilevelJuMP.CI}())
    end

    # upper_primal_to_lower_primal = Dict{BilevelJuMP.VI, BilevelJuMP.VI}()

    UL_obj_type = MOI.get(upper, MOI.ObjectiveFunctionType())
    upper_obj_func_quad_terms = MOI.get(
        upper, MOI.ObjectiveFunction{UL_obj_type}()).quadratic_terms

    Threads.@threads for upper_dual_var_idx in collect(eachindex(upper_var_to_lower_ctr))
        id = Threads.threadid()
        for term in upper_obj_func_quad_terms

            if upper_dual_var_idx == term.variable_1
                if term.variable_2 in values(upper_to_lower_var_indices)
                    lower_primal_var_idx = upper_to_lower_var_indices[term.variable_2]
                    push!(A_N[id], lower_primal_var_idx.value)
                    # upper_primal_to_lower_primal[term.variable_2] = lower_primal_var_idx
                    # upper_dual_to_quad_term[upper_dual_var_idx] = [term]  # will overwrite entry if upper_dual is multiplied by more than one lower variable in the upper objective, needs to list of pairs, muliple cases here
                    # upper_dual_to_quad_term is only used to find A_jn, the coefs in the upper obj of lambda_j and yn, could instead make matrix 
                    # upper_dual_to_quad_term[upper_dual_var_idx][upper_to_lower_var_indices[term.variable_2]] = term.coefficient, which changes to variable_1 in elseif below
                    if !haskey(upper_dual_to_quad_term[id], upper_dual_var_idx)
                        upper_dual_to_quad_term[id][upper_dual_var_idx] = Dict(
                            upper_to_lower_var_indices[term.variable_2] => term.coefficient

                        )
                    else
                        upper_dual_to_quad_term[id][upper_dual_var_idx][upper_to_lower_var_indices[term.variable_2]] = term.coefficient
                    end
                    if !haskey(upper_dual_to_lower_primal[id], upper_dual_var_idx)
                        upper_dual_to_lower_primal[id][upper_dual_var_idx] = [lower_primal_var_idx]
                    else
                        push!(upper_dual_to_lower_primal[id][upper_dual_var_idx], lower_primal_var_idx)
                    end
                    lower_primal_var_to_lower_con[id][lower_primal_var_idx] = upper_var_to_lower_ctr[upper_dual_var_idx]
                end

            elseif upper_dual_var_idx == term.variable_2
                if term.variable_1 in values(upper_to_lower_var_indices)
                    lower_primal_var_idx = upper_to_lower_var_indices[term.variable_1]
                    push!(A_N[id], lower_primal_var_idx.value)
                    # upper_primal_to_lower_primal[term.variable_1] = lower_primal_var_idx
                    if !haskey(upper_dual_to_quad_term[id], upper_dual_var_idx)
                        upper_dual_to_quad_term[id][upper_dual_var_idx] = Dict(
                            upper_to_lower_var_indices[term.variable_1] => term.coefficient

                        )
                    else
                        upper_dual_to_quad_term[id][upper_dual_var_idx][upper_to_lower_var_indices[term.variable_1]] = term.coefficient
                    end
                    if !haskey(upper_dual_to_lower_primal[id], upper_dual_var_idx)
                        upper_dual_to_lower_primal[id][upper_dual_var_idx] = [lower_primal_var_idx]
                    else
                        push!(upper_dual_to_lower_primal[id][upper_dual_var_idx], lower_primal_var_idx)
                    end
                    lower_primal_var_to_lower_con[id][lower_primal_var_idx] = upper_var_to_lower_ctr[upper_dual_var_idx]
                end
            end
        end
    end

    upper_dual_to_quad_term = mergewith(merge, upper_dual_to_quad_term...)
    upper_dual_to_lower_primal = mergewith(vcat, upper_dual_to_lower_primal...)
    lower_primal_var_to_lower_con = merge(lower_primal_var_to_lower_con...)
    return unique(vcat(finish!.(A_N)...)), upper_dual_to_quad_term, upper_dual_to_lower_primal, lower_primal_var_to_lower_con
end


function get_lower_obj_coefs_of_upper_times_lower_primals(
    lower, 
    lower_var_indices_of_upper_vars, 
    A_N, 
    lower_to_m_idxmap,
    )
    AB_N = Int[]
    nvars = MOI.get(lower, MOI.NumberOfVariables())
    # B is in LL objective: x^T B y
    B = spzeros(nvars, nvars)

    ks = collect(keys(lower_to_m_idxmap))
    lower_var_indices = collect(filter(k -> typeof(k)==MOI.VariableIndex , ks))
    lower_only_vars = setdiff(lower_var_indices, lower_var_indices_of_upper_vars)

    # get lower objective terms for finding cost coefficients later
    lower_obj_type = MOI.get(lower, MOI.ObjectiveFunctionType())
    lower_obj = MOI.get(lower, MOI.ObjectiveFunction{lower_obj_type}())
    lower_obj_quad_terms = nothing
    lower_obj_terms = nothing
    lower_obj_type_handled = true

    if lower_obj_type == MOI.ScalarQuadraticFunction{Float64}
        lower_obj_quad_terms = lower_obj.quadratic_terms
        lower_obj_terms = lower_obj.affine_terms
    elseif lower_obj_type == MOI.ScalarAffineFunction{Float64}
        lower_obj_terms = lower_obj.terms
    else
        lower_obj_type_handled = false
    end

    # fill the set AB_N = {n in A_N : ∃ m ∈ M s.t. B_mn ≠ 0} and the B matrix (x^T B y in LL objective)
    # NOTE B is assumed to index by upper, lower var index (m,n) indices,
    # (which are not necessarily in the order entered by user).
    if !isnothing(lower_obj_quad_terms)  # check for values in AB_N, 
        for term in lower_obj_quad_terms
            if term.variable_1 in lower_var_indices_of_upper_vars # UL var
                if term.variable_2.value in A_N  # LL var
                    push!(AB_N, term.variable_2.value)  # AB_N is not empty
                end
                if term.variable_2 in lower_only_vars
                    B[term.variable_1.value, term.variable_2.value] = 
                        get_coef(term.variable_1, term.variable_2, lower_obj_quad_terms)
                end 
            elseif term.variable_2 in lower_var_indices_of_upper_vars  # UL var
                if term.variable_1.value in A_N  # LL var
                    push!(AB_N, term.variable_1.value) # AB_N is not empty
                end
                if term.variable_1 in lower_only_vars
                    B[term.variable_2.value, term.variable_1.value] = 
                        get_coef(term.variable_1, term.variable_2, lower_obj_quad_terms)
                end 
            end
        end
    end
    return AB_N, B, lower_obj_terms, lower_obj_type_handled
end


function linear_terms_for_empty_AB(
        lower,
        upper_var_to_lower_ctr,
        bilinear_upper_dual_to_lower_primal,
        V,
        w,
        bilinear_upper_dual_to_quad_term,
        lower_obj_terms,
        lower_to_m_idxmap,
        lower_primal_dual_map,
        lower_dual_idxmap
    )
    linearizations = Vector{MOI.ScalarAffineTerm}()
    con_type = MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64}}
    I, J, vals = findnz(V)

    for (upper_var, lower_con) in upper_var_to_lower_ctr  # TODO thread
        j = lower_con.value
        for lower_var in bilinear_upper_dual_to_lower_primal[upper_var]
            n = lower_var.value

            J_j, N_n, redundant_vals = find_connected_rows_cols_cached(V, I, J, vals, j, n, skip_1st_col_check=false)
            if redundant_vals
                return nothing
            end

            A_jn = bilinear_upper_dual_to_quad_term[upper_var][lower_var]
            V_jn = V[j,n]
            
            for j_prime in J_j
                lower_con_index = con_type(j_prime)
                lower_dual_var = lower_primal_dual_map.primal_con_dual_var[lower_con_index][1]

                push!(linearizations,
                    MOI.ScalarAffineTerm(A_jn / V_jn * w[j_prime], lower_dual_idxmap[lower_dual_var])  
                )
            end

            # TODO assert that lower level constraints in upper_var_to_lower_ctr are linear
            
            num_vars = MOI.get(lower, MOI.NumberOfVariables())
            for n_prime in N_n
                if n_prime > num_vars continue end  # TODO do we need to add slack variables?
                lower_var = MOI.VariableIndex(n_prime)
                # lower primal * lower cost
                lower_var_cost_coef = BilevelJuMP.get_coef(lower_var, lower_obj_terms)
                push!(linearizations,
                    MOI.ScalarAffineTerm(-A_jn / V_jn * lower_var_cost_coef, lower_to_m_idxmap[lower_var])
                )
                # variable bound * dual variable
                low_bound, upp_bound = MOIU.get_bounds(lower, Float64, lower_var) # yl[n_prime], yu[n_prime] #
                lo_bound_index = MOI.ConstraintIndex{MOI.VariableIndex, MOI.GreaterThan{Float64}}(lower_var.value)
                up_bound_index = MOI.ConstraintIndex{MOI.VariableIndex, MOI.LessThan{Float64}}(lower_var.value)
                low_dual = get(lower_primal_dual_map.primal_con_dual_var, lo_bound_index, [nothing])[1]
                upp_dual = get(lower_primal_dual_map.primal_con_dual_var, up_bound_index, [nothing])[1]

                # have to use opposite signs of paper for these terms (b/c Dualization sets variable bound dual variables to be non-positive?)
                if low_bound != -Inf && !isnothing(low_dual)
                    push!(linearizations, MOI.ScalarAffineTerm(-A_jn / V_jn * low_bound, lower_dual_idxmap[low_dual]))
                end
                if upp_bound != Inf && !isnothing(upp_dual) # TODO add a big number in place of Inf ?
                    push!(linearizations, MOI.ScalarAffineTerm( A_jn / V_jn * upp_bound, lower_dual_idxmap[upp_dual]))
                end
            end
        end
    end

    return linearizations
end


function linear_terms_for_non_empty_AB(
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
        lower_dual_idxmap
    )
    linearizations = PushVector{MOI.ScalarAffineTerm}()
    con_type = MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64}}
    I, J, vals = findnz(V)

    for (upper_var, lower_con) in upper_var_to_lower_ctr # TODO Thread
        j = lower_con.value

        for lower_var in bilinear_upper_dual_to_lower_primal[upper_var]

            A_jn = bilinear_upper_dual_to_quad_term[upper_var][lower_var]
            if isapprox(A_jn, 0.0; atol=1e-12)
                continue
            end
            n = lower_var.value
            # this call is same as in get_all_connected_rows_cols, hence memoization should speed things up
            rows, cols, redundant_vals = find_connected_rows_cols_cached(V, I, J, vals, j, n, skip_1st_col_check=true)
            # rows is set J_j, cols is set N_n
            if redundant_vals
                return nothing
            end

            V_jn = V[j,n]
            p = A_jn / V_jn
            for r in rows
                if isapprox(w[r], 0.0; atol=1e-12)
                    continue
                end
                lower_con_index = con_type(r)
                lower_dual_var = lower_primal_dual_map.primal_con_dual_var[lower_con_index][1]

                push!(linearizations,
                    MOI.ScalarAffineTerm(p*w[r], lower_dual_idxmap[lower_dual_var])  
                )
            end

            # TODO assert that lower level constraints in upper_var_to_lower_ctr are linear
            cols = setdiff(cols, A_N)
            num_vars = MOI.get(lower, MOI.NumberOfVariables())
            for c in cols
                if c > num_vars continue end  # TODO do we need to add slack variables?
                lower_var = MOI.VariableIndex(c)
                # lower primal * lower cost
                lower_var_cost_coef = BilevelJuMP.get_coef(lower_var, lower_obj_terms)
                push!(linearizations,
                    MOI.ScalarAffineTerm(-p*lower_var_cost_coef, lower_to_m_idxmap[lower_var])
                )
                # variable bound * dual variable
                low_bound, upp_bound = MOIU.get_bounds(lower, Float64, lower_var) # yl[n_prime], yu[n_prime] #
                lo_bound_index = MOI.ConstraintIndex{MOI.VariableIndex, MOI.GreaterThan{Float64}}(lower_var.value)
                up_bound_index = MOI.ConstraintIndex{MOI.VariableIndex, MOI.LessThan{Float64}}(lower_var.value)
                low_dual = get(lower_primal_dual_map.primal_con_dual_var, lo_bound_index, [nothing])[1]
                upp_dual = get(lower_primal_dual_map.primal_con_dual_var, up_bound_index, [nothing])[1]

                # have to use opposite signs of paper for these terms (b/c Dualization sets variable bound dual variables to be non-positive?)
                if low_bound != -Inf && !isapprox(low_bound, 0.0; atol=1e-12) && !isnothing(low_dual)
                    push!(linearizations, MOI.ScalarAffineTerm(-p * low_bound, lower_dual_idxmap[low_dual]))
                end
                if upp_bound != Inf && !isapprox(upp_bound, 0.0; atol=1e-12) && !isnothing(upp_dual) # TODO add a big number in place of Inf ?
                    push!(linearizations, MOI.ScalarAffineTerm( p * upp_bound, lower_dual_idxmap[upp_dual]))
                end
            end
        end
    end

    return finish!(linearizations)
end


function check_condition_1(J_U::AbstractVector{Int}, U::AbstractMatrix)
    # Condition 1: U_jm = 0 ∀ j ∈ J_U, ∀ m ∈ M
    met_condition = true
    for j in J_U, m in 1:size(U,2)
        if U[j,m] ≠ 0
            met_condition = false
            @warn("Condition 1 not met: at least one connected lower constraint contains upper level variables.")
            break
        end
    end
    met_condition
end


function check_condition_2(N_U::AbstractVector{Int}, U::AbstractMatrix, B::AbstractMatrix)
    # Condition 2: B_mn = 0 ∀ m ∈ M, ∀ n ∈ N_U
    met_condition = true
    for m in 1:size(U,1), n in N_U
        if B[m,n] ≠ 0
            met_condition = false
            @warn("Condition 2 not met: at least one connected lower variable is multiplied with an upper variable in the lower objective.")
            break
        end
    end
    met_condition
end


function check_condition_2prime(
    N_U::AbstractVector{Int}, A_N::AbstractVector{Int}, 
    U::AbstractMatrix, B::AbstractMatrix
    )
    # Condition 2': B_mn = 0 ∀ m ∈ M, ∀ n ∈ N_U \ A_N
    met_condition = true
    for m in 1:size(U,2), n in setdiff(N_U, A_N)
        if B[m,n] ≠ 0
            met_condition = false
            @warn("Condition 2' not met: at least one connected lower variable (that is not in the upper objective with a lower dual) is multiplied with an upper variable in the lower objective.")
            break
        end
    end
    met_condition
end


function check_condition_3(A_N::AbstractVector{Int}, V::AbstractMatrix, lower_primal_var_to_lower_con)
    # Condition 3: A_N \ n ⊆ N_n ∀ n ∈ A_n
    I, J, vals = findnz(V)
    met_condition = true
        j = lower_primal_var_to_lower_con[MOI.VariableIndex(n)].value
        # this call is same as in get_all_connected_rows_cols, hence memoization should speed things up
        _, N_n, _ = find_connected_rows_cols_cached(V, I, J, vals, j, n, skip_1st_col_check=true)
        # NOTE not handling redundant_vals here b/c goal is to first check conditions 
        # (redundant_vals handled when determining linearizations)
        if !(issubset(setdiff(A_N, n), N_n))
            met_condition = false
            @warn("Condition 3 not met: at least one lower variable from the upper objective bilinear terms is not connected to the other lower variables in the upper bilinear terms.")
            break
        end
    end
    met_condition
end


function check_condition_4(A_N::AbstractVector{Int}, V::AbstractMatrix, upper_var_to_lower_ctr, bilinear_upper_dual_to_lower_primal)
    # Condition 4: V_j'n = 0 ∀ j' ∈ J \ {j}, ∀ (j,n) ∈ A
    met_condition = true
    J = 1:size(V,1)
    for (upper_var, lower_con) in upper_var_to_lower_ctr  # thread?
        j = lower_con.value
        for lower_var in bilinear_upper_dual_to_lower_primal[upper_var]
            n = lower_var.value
            if !(n in A_N) continue end  # TODO can we pass in a better set to loop over?
            for j_prime in setdiff(J, j)
                if V[j_prime, n] ≠ 0
                    met_condition = false
                    @warn("Condition 4 not met: at least one lower variable from the upper objective bilinear terms is in more than one lower constraint.")
                    @goto outer_break
                end
            end
        end
    end
    @label outer_break
    return met_condition
end


function check_condition_5(A_N::AbstractVector{Int}, V::AbstractMatrix, upper_var_to_lower_ctr, bilinear_upper_dual_to_lower_primal, bilinear_upper_dual_to_quad_term)
    # Condition 5: A_jn = p V_jn ∀ (j,n) ∈ A (p is proportionality constant)
    met_condition = true
    p = nothing
    for (upper_var, lower_con) in upper_var_to_lower_ctr  # thread?
        j = lower_con.value
        for lower_var in bilinear_upper_dual_to_lower_primal[upper_var]
            n = lower_var.value
            if !(n in A_N) continue end  # TODO can we pass in a better set to loop over?
            A_jn = bilinear_upper_dual_to_quad_term[upper_var][lower_var]
            V_jn = V[j,n]
            if !(isnothing(p)) && !(isapprox(p, A_jn / V_jn, atol=1e-5))
                met_condition = false
                @warn("Condition 5 not met: at least one of the ratios of the upper objective bilinear coefficient to lower level constraint coefficient is not equal to the other ratios.")
            end
            p = A_jn / V_jn
        end
    end
    return met_condition
end


"""
    check_empty_AB_N_conditions(J_U, U, N_U, B)

Check two required conditions for linearizing bilinear terms in UL of the form λj * yn when there
    are no xm*yn in the LL objective (when AB_N is empty set).
"""
function check_empty_AB_N_conditions(J_U, U, N_U, B)
    # Condition 1: U_jm = 0 ∀ j ∈ J_U, ∀ m ∈ M
    met_condition_1 = check_condition_1(J_U, U)

    # Condition 2: B_mn = 0 ∀ m ∈ M, ∀ n ∈ N_U
    met_condition_2 = check_condition_2(N_U, U, B)

    if met_condition_1 && met_condition_2
        return true
    end

    return false
end


"""
    check_non_empty_AB_N_conditions(J_U, U, N_U, A_N, B, V, lower_primal_var_to_lower_con, 
        upper_var_to_lower_ctr, bilinear_upper_dual_to_quad_term, bilinear_upper_dual_to_lower_primal)

Check five required conditions for linearizing bilinear terms in UL of the form λj * yn when there
    are xm*yn in the LL objective (when AB_N is not empty).
"""
function check_non_empty_AB_N_conditions(J_U, U, N_U, A_N, B, V, lower_primal_var_to_lower_con, 
    upper_var_to_lower_ctr, bilinear_upper_dual_to_quad_term, bilinear_upper_dual_to_lower_primal)
    # Condition 1: U_jm = 0 ∀ j ∈ J_U, ∀ m ∈ M
    met_condition_1 = check_condition_1(J_U, U)

    # Condition 2': B_mn = 0 ∀ m ∈ M, ∀ n ∈ N_U \ A_N
    met_condition_2 = check_condition_2prime(N_U, A_N, U, B)

    # Condition 3: A_N \ n ⊆ N_n ∀ n ∈ A_n
    met_condition_3 = check_condition_3(A_N, V, lower_primal_var_to_lower_con)

    # Condition 4: V_j'n = 0 ∀ j' ∈ J \ {j}, ∀ (j,n) ∈ A
    met_condition_4 = check_condition_4(A_N, V, upper_var_to_lower_ctr, 
        bilinear_upper_dual_to_lower_primal)
    # Condition 5: A_jn = p V_jn ∀ (j,n) ∈ A (p is proportionality constant)
    met_condition_5 = check_condition_5(A_N, V, upper_var_to_lower_ctr, 
        bilinear_upper_dual_to_lower_primal, bilinear_upper_dual_to_quad_term)

    if met_condition_1 && met_condition_2 && met_condition_3 && met_condition_4 && 
        met_condition_5
        return true
    end

    return false
end


"""
    find_blocks(V::AbstractMatrix{<:Real}, U::AbstractMatrix{<:Real})

Find the blocks in the joint matrix [U V]

For example, given:

[U V] = | 1 0 0 1 0 0 |
        | 0 1 0 0 1 0 |
        | 0 1 0 0 1 0 |

rows = [
    [1], [2,3]
]
cols = [
    [1, 4], [2,5]
]

NOTE that rows/cols with all zeros are not included in the return values.
"""
function find_blocks(V::AbstractMatrix{<:Real}, U::AbstractMatrix{<:Real})

    # have to create nthreads arrays and then combine them at the end (to be thread safe)
    num_blocks = 0
    rows, cols = PushVector{Vector}(), PushVector{Vector}()
    rows_connected = PushVector{Int}()

    # starting with first row of V, find all connected values. 
    # If all rows are connected then there is only one block. 
    # Else, search the remaining unconnected rows until all rows are accounted for.
    nrows = size(V,1)
    UV = U + V
    for r in 1:nrows
        if r in rows_connected continue end  # cannot multithread this for loop b/c of this check
        c = findfirst(!iszero, UV[r,:])
        if !isnothing(c)
            num_blocks += 1
            rs, cs = find_connected_rows_cols(UV, r, c; skip_1st_col_check=true, finding_blocks=true)
            # not worried about redundant_vals here; it is checked when forming linearizations
            push!(rows, rs)
            push!(cols, cs)
            push!(rows_connected, rs...)
        end
    end
    @debug("Found $(num_blocks) block(s) in V.")
    return num_blocks, finish!(rows), finish!(cols)
end


function is_model_in_standard_form(m::MOI.ModelLike)
    model_con_types = Set(MOI.get(m, MOI.ListOfConstraintTypesPresent()))
    standard_form_con_types = Set([
        (MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64}),
        (MOI.VariableIndex, MOI.GreaterThan{Float64}),
        (MOI.VariableIndex, MOI.LessThan{Float64})
    ])
    issubset(model_con_types, standard_form_con_types)
end


function get_all_connected_rows_cols(upper_var_to_lower_ctr, bilinear_upper_dual_to_lower_primal, V, AB_N) # TODO rm V arg

    # have to create nthreads arrays and then combine them at the end (to be thread safe)
    J_Us = Vector{PushVector{Int}}()
    N_Us = Vector{PushVector{Int}}()
    for _ in 1:Threads.nthreads()
        push!(J_Us, PushVector{Int}())
        push!(N_Us, PushVector{Int}())
    end
    I, J, vals = findnz(V)

    Threads.@threads for upper_var in collect(eachindex(upper_var_to_lower_ctr))  # equivalent to set A with pairs (j,n) : A_jn ≠ 0
        lower_con = upper_var_to_lower_ctr[upper_var]
        if !(upper_var in keys(bilinear_upper_dual_to_lower_primal)) continue end  # user defined DualOf but did not use it in UL objective
        j = lower_con.value
        for lower_var in bilinear_upper_dual_to_lower_primal[upper_var]
            n = lower_var.value
            rows, cols = find_connected_rows_cols_cached(V, I, J, vals, j, n, skip_1st_col_check=!(isempty(AB_N)))
            push!(J_Us[Threads.threadid()], rows...)
            push!(N_Us[Threads.threadid()], cols...)
        end
    end
    return unique(vcat(finish!.(J_Us)...)), unique(vcat(finish!.(N_Us)...))
end


"""
    main_linearization(
        lower, 
        upper, 
        upper_var_to_lower_ctr, 
        upper_to_lower_var_indices, 
        lower_var_indices_of_upper_vars, 
        lower_to_m_idxmap, 
        lower_primal_dual_map, 
        lower_dual_idxmap
    )

The main logic for linearizing bilinear terms of lower level dual and primal variables in the upper
level model.
"""
function main_linearization(
        m,
        lower, 
        upper, 
        upper_var_to_lower_ctr, 
        upper_to_lower_var_indices, 
        lower_var_indices_of_upper_vars, 
        lower_to_m_idxmap, 
        upper_to_m_idxmap,
        lower_primal_dual_map, 
        lower_dual_idxmap
    )
    @info """starting main linearization at $(Dates.format(now(), "HH:MM:SS"))"""
    if MOI.get(upper, MOI.ObjectiveFunctionType()) <: MOI.ScalarQuadraticFunction &&
        !isempty(upper_var_to_lower_ctr)

        @assert MOI.get(upper, MOI.ObjectiveSense()) == MOI.MIN_SENSE
        @assert MOI.get(lower, MOI.ObjectiveSense()) == MOI.MIN_SENSE
        # TODO switch signs with MAX sense
        
        linearize = true
        # check lower constraint types and if not just equality and singlevariable bounds then linearize = false and @warn

        @info """starting is_model_in_standard_form at $(Dates.format(now(), "HH:MM:SS"))"""
        if !(is_model_in_standard_form(lower))
            linearize = false
            @warn("The lower model must be in standard form to linearize bilinear terms. Skipping linearization process.")
        else
            @info """starting check_upper_objective_for_bilinear_linearization at $(Dates.format(now(), "HH:MM:SS"))"""
            A_N, bilinear_upper_dual_to_quad_term, bilinear_upper_dual_to_lower_primal, lower_primal_var_to_lower_con = 
                check_upper_objective_for_bilinear_linearization(upper, upper_to_lower_var_indices, upper_var_to_lower_ctr)
            if isempty(A_N)
                @debug("No bilinear products of lower level dual and primal variables found in upper level objective. Skipping linearization process.")
                linearize = false
            else
                @info """starting get_lower_obj_coefs_of_upper_times_lower_primals at $(Dates.format(now(), "HH:MM:SS"))"""
                AB_N, B, lower_obj_terms, lower_obj_type_handled = 
                    get_lower_obj_coefs_of_upper_times_lower_primals(lower, lower_var_indices_of_upper_vars, A_N, lower_to_m_idxmap)
                if !lower_obj_type_handled
                    @warn("Linearizing bilinear terms does not handle lower level objective type $(MOI.get(lower, MOI.ObjectiveFunctionType())). Skipping linearization process.")
                    linearize = false
                end
            end
        end
    else
        @debug("Upper objective must be quadratic and there must lower dual variables in the upper model to linearize bilinear terms. Skipping linearization process.")
        linearize = false
    end

    if linearize
        @info """starting standard_form at $(Dates.format(now(), "HH:MM:SS"))"""
        U, V, w = standard_form(lower, upper_var_indices=lower_var_indices_of_upper_vars)
        upper_obj_func_quad_terms = MOI.get(upper, MOI.ObjectiveFunction{MOI.get(upper, MOI.ObjectiveFunctionType())}()).quadratic_terms
        linearizations = nothing
        m_objective = MOI.get(m, MOI.ObjectiveFunction{MOI.get(m, MOI.ObjectiveFunctionType())}())
        bilinear_upper_quad_term_to_m_quad_term = Dict{MOI.ScalarQuadraticTerm, MOI.ScalarQuadraticTerm}()

        @info """starting loop over m_objective.quadratic_terms at $(Dates.format(now(), "HH:MM:SS"))"""
        for term in m_objective.quadratic_terms
            mset = Set([term.variable_1, term.variable_2])
            for upper_term in upper_obj_func_quad_terms
                uset = Set([
                    upper_to_m_idxmap[upper_term.variable_1],
                    upper_to_m_idxmap[upper_term.variable_2]
                ])
                if uset == mset
                    bilinear_upper_quad_term_to_m_quad_term[upper_term] = term
                end
            end
        end

        # TODO check for integer x * continuous y, for now assuming continuous x conditions
        @info """starting get_all_connected_rows_cols at $(Dates.format(now(), "HH:MM:SS"))"""
        J_U, N_U = get_all_connected_rows_cols(upper_var_to_lower_ctr, bilinear_upper_dual_to_lower_primal, V, AB_N)
        #= 
            Case without x_m * y_n in LL objective for all y_n in A_N (set of bilinear UL objective terms of form λ_j * y_n)
        =#
        if isempty(AB_N)
            @debug("set AB_N is empty")

            conditions_passed = check_empty_AB_N_conditions(J_U, U, N_U, B)

            if conditions_passed
                linearizations = linear_terms_for_empty_AB(
                    lower,
                    upper_var_to_lower_ctr,
                    bilinear_upper_dual_to_lower_primal,
                    V,
                    w,
                    bilinear_upper_dual_to_quad_term,
                    lower_obj_terms,
                    lower_to_m_idxmap,
                    lower_primal_dual_map,
                    lower_dual_idxmap
                )
                if isnothing(linearizations)
                    @warn("Unable to linearize bilinear terms due to underdertermined system of equations. Skipping linearization process.")
                end
            else
                @warn("Required conditions for linearization not met. Skipping linearization process.")
            end
        else  # AB_N is not empty
            @debug("set AB_N is NOT empty")
            
            # TODO input flag for checking for blocks? (to save time)
            
            @info """starting find_blocks at $(Dates.format(now(), "HH:MM:SS"))"""
            num_blocks, rows, cols = find_blocks(V, U)

            conditions_passed = Bool[]
            nrows, ncols = size(V)
            
            @info """starting condition checks at $(Dates.format(now(), "HH:MM:SS"))"""
            for n in 1:num_blocks
                # TODO can skip blocks that are not linked to bilinear terms?
                Vblock = spzeros(nrows, ncols)
                Vblock[rows[n],:] = V[rows[n],:]
                check = check_non_empty_AB_N_conditions(
                    intersect(J_U, rows[n]), U, intersect(N_U, cols[n]), intersect(A_N, cols[n]), B, Vblock, 
                    lower_primal_var_to_lower_con, upper_var_to_lower_ctr,
                    bilinear_upper_dual_to_quad_term, 
                    bilinear_upper_dual_to_lower_primal)
                push!(conditions_passed, check)
            end
            # recover memory
            rows, cols = nothing, nothing
            @debug("Done looping over V blocks")
            
            if all(conditions_passed)

                @info """starting linear_terms_for_non_empty_AB at $(Dates.format(now(), "HH:MM:SS"))"""
                linearizations = linear_terms_for_non_empty_AB(
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
                    lower_dual_idxmap
                )
                if isnothing(linearizations)
                    @warn("Unable to linearize bilinear terms due to underdertermined system of equations. Skipping linearization process.")
                end
            else
                @warn("Required conditions for linearization not met. Skipping linearization process.")
            end

        end

        # LINEARIZATION

        if !(isnothing(linearizations))
            
            @info """starting linearizations at $(Dates.format(now(), "HH:MM:SS"))"""
            # set m's objective by replacing quadratic terms with linearizations
            mobj = deepcopy(m_objective)
            quadratic_terms = mobj.quadratic_terms  # TODO keep quadratic terms that have not been linearized
            c = mobj.constant
            affine_terms = mobj.affine_terms
            cvb = collect(values(bilinear_upper_quad_term_to_m_quad_term))
            new_objective = deepcopy(m_objective)
            if Set(cvb) == Set(quadratic_terms)
                @debug("Replacing bilinear lower dual * lower primal terms in upper objective with linear terms.")
                new_objective = MOI.ScalarAffineFunction{Float64}(
                    append!(affine_terms, linearizations),
                    c
                )
                MOI.set(m, MOI.ObjectiveFunction{MOI.ScalarAffineFunction}(), new_objective)
            else
                @warn("Unable to linearize bilinear terms due to mis-matched quadratic terms in the upper level and single level models. Skipping linearization process.")
            end
        end
    end # if linearize
    @info """done linearizing at $(Dates.format(now(), "HH:MM:SS"))"""
end
