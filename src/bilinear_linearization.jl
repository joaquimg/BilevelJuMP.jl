"""
    non_zero_idxs_except_one(v::AbstractVector, idx::Int)

Return indices of all the non-zero values in v except idx
"""
function non_zero_idxs_except_one(v::AbstractVector, idx::Int)
    idxs = findall(!iszero, v)
    if !isempty(idxs)
        deleteat!(idxs, findfirst(x->x==idx, idxs))
    end
    return idxs
end

"""
Given a row, col in an array A find all the connected rows and cols starting with searching col, 
where "connected" rows and cols are those with non-zero values.
Search is stopped if any redundant rows or cols are added to the output arrays 
    (which indicates that there is a loop in the connections).
"""
function recursive_col_search(A::AbstractArray, row::Int, col::Int, 
    rows::Vector{Int}, cols::Vector{Int})
    rs = non_zero_idxs_except_one(A[:, col], row)
    if any(r in rows for r in rs)
        rr = intersect(rs, rows)
        @warn("Returning from recursive_col_search due to redundant row! ($rr)")
        return rows, cols
    end
    push!(rows, rs...)
    for r in rs
        cs = non_zero_idxs_except_one(A[r, :], col)
        if any(c in cols for c in cs)
            cc = intersect(cs, cols)
            @warn("Returning from recursive_col_search due to redundant column! ($cc)")
            return rows, cols
        end
        push!(cols, cs...)
        for c in cs
            recursive_col_search(A, r, c, rows, cols)
        end
    end
    return rows, cols
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
function find_connected_rows_cols(A::AbstractArray, row::Int, col::Int; skip_1st_col_check=false)
    @assert A[row, col] != 0

    # step 1 check if all non-zeros in A[:, col], if so the dual constraint gives linearization
    # NOTE: may not want to use this linearization method: if the lower level variable at row,col is
    # in the lower level objective * an upper level variable then one does not get a linearization
    # (since the Dk equation has the lower level cost * the lower level variable)
    if length(findall(!iszero, A[:, col])) == 1 & !skip_1st_col_check
        return [], [col]
    end
    # step 2 add 1st row and any other non-zero columns
    rows = [row]
    cols_to_check = non_zero_idxs_except_one(A[row, :], col)
    cols = copy(cols_to_check)
    # step 3 recursive search to find all connections
    for c in cols_to_check
        rows_to_add, cols_to_add = recursive_col_search(A, row, c, Int[], Int[])
        push!(rows, rows_to_add...)
        push!(cols, cols_to_add...)
    end
    # sort!(rows)
    # sort!(cols)
    return rows, cols
end


function get_coef(var::MOI.VariableIndex, safts::Vector{MOI.ScalarAffineTerm{R}}) where R <: Real
    coef = 0
    for saft in safts
        if var == saft.variable_index
            coef = saft.coefficient
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
	f = spzeros(nrows)

	for (r, ci) in enumerate(constraint_indices)
		con_func = MOI.get(m, MOI.ConstraintFunction(), ci)
		for term in con_func.terms
			E[r, term.variable_index.value] = term.coefficient
		end
		f[r] = MOI.get(m, MOI.ConstraintSet(), ci).lower
	end
	return E, f
end


# can replace the following SingleVariable limits with an MOI function for getting bounds?
function get_coef_matrix_and_rhs_vec(m, 
	constraint_indices::Array{
		MOI.ConstraintIndex{
			MOI.SingleVariable, 
			MOI.GreaterThan{Float64}
		}, 1}
	)
	f = -Inf*ones(MOI.get(m, MOI.NumberOfVariables()))

	for ci in constraint_indices
		var_index = MOI.get(m, MOI.ConstraintFunction(), ci)
		f[var_index.variable.value] = MOI.get(m, MOI.ConstraintSet(), ci).lower
	end
	return f
end


function get_coef_matrix_and_rhs_vec(m, 
	constraint_indices::Array{MOI.ConstraintIndex{
			MOI.SingleVariable, 
			MOI.LessThan{Float64}},
		1}
	)
	d = Inf*ones(MOI.get(m, MOI.NumberOfVariables()))

	for ci in constraint_indices
		var_index = MOI.get(m, MOI.ConstraintFunction(), ci)
		d[var_index.variable.value] = MOI.get(m, MOI.ConstraintSet(), ci).upper
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
	d = spzeros(nrows)
	for (r, ci) in enumerate(constraint_indices)
		con_func = MOI.get(m, MOI.ConstraintFunction(), ci)
		for term in con_func.terms
			C[r, term.variable_index.value] = term.coefficient
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
            V[r, term.variable_index.value] = term.coefficient
        end
        w[r] = con_set.value
    end
    return V, w
end


"""

    given Dx + Ey = f, Px + Qy ≤ r, Sx + Ty ≥ u
    collect all of y bounds into 
    yl ≤ y ≤ yu
    and put all inequality constraints into equalities with slack variables s:
    Ux + V [y; s] = [w; b]

	maybe also return Model built in standard form
"""
function standard_form(m; upper_var_indices=Vector{MOI.VariableIndex}())
	nvars = MOI.get(m, MOI.NumberOfVariables())
	con_types = MOI.get(m, MOI.ListOfConstraints())

    n_equality_cons = 0
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
    # V = rest of A, but how to map new variable indices to LL variable indices?
    # do the indices get set by the order in which variables are added? so when we build the standard form model does adding y first make y the first N variables?
    # if so, then we have to map those indices to the columns in A / the indices in LL model
	# TODO SingleVariable, EqualTo 
    # TODO what is best way to map variable indices in the standard form model to the LL model?
    #=
    Maybe best approach is to build single matrix A[x;y] = b s.t. A's columns match LL model variable indices,
    then split A into U and V while creating variable index map.
    This approach requires adding columns to A for the slack variables, so those columns should start at nvars+1
    =#
	

    #=
    For each ScalarAffineFunction ≥ or ≤ Float64 we add a slack variable and make the constraint
    an EqualTo{Float64}
    =#
    n_lessthan_cons = 0
	if (MOI.ScalarAffineFunction{Float64}, MOI.LessThan{Float64}) in con_types

		lt_con_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.ScalarAffineFunction{Float64}, 
			MOI.LessThan{Float64}
		}());

		C, d = get_coef_matrix_and_rhs_vec(m, lt_con_indices)
	else
		C, d = spzeros(0, nvars), spzeros(0)
	end
	
    n_greaterthan_cons = 0
	if (MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64}) in con_types

		gt_con_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.ScalarAffineFunction{Float64}, 
			MOI.GreaterThan{Float64}
		}());

		E, f = get_coef_matrix_and_rhs_vec(m, gt_con_indices)
	else
		E, f = spzeros(0, nvars), spzeros(0)
	end
	
    #=
    For each SingleVariable GreaterThan{Float64} or LessThan{Float64} we fill in the bounds
    yl and yu
    =#
    yl = -Inf*ones(MOI.get(m, MOI.NumberOfVariables()))
	if (MOI.SingleVariable, MOI.GreaterThan{Float64}) in con_types

		singleVar_gt_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.SingleVariable, 
			MOI.GreaterThan{Float64}
		}());

		yl = get_coef_matrix_and_rhs_vec(m, singleVar_gt_indices)
	end
	
    yu = Inf*ones(MOI.get(m, MOI.NumberOfVariables()))
	if (MOI.SingleVariable, MOI.LessThan{Float64}) in con_types

		singleVar_lt_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.SingleVariable, 
			MOI.LessThan{Float64}
		}());

		yu = get_coef_matrix_and_rhs_vec(m, singleVar_lt_indices)
	end

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
    n_vars = size(A,2)
    V = spzeros(n_equality_cons + n_greaterthan_cons + n_lessthan_cons, 
                n_vars + n_greaterthan_cons + n_lessthan_cons)
    V[1:n_equality_cons, 1:n_vars] = A

    V[n_equality_cons+1 : n_equality_cons+n_lessthan_cons, 1 : n_vars] = C
    V[n_equality_cons+1 : n_equality_cons+n_lessthan_cons, n_vars+1 : n_vars+n_lessthan_cons] = Matrix(I, n_lessthan_cons, n_lessthan_cons)

    V[n_equality_cons+n_lessthan_cons+1 : n_equality_cons+n_greaterthan_cons+n_lessthan_cons, 1:n_vars] = E
    V[n_equality_cons+n_lessthan_cons+1 : n_equality_cons+n_greaterthan_cons+n_lessthan_cons, 
      n_vars+n_lessthan_cons+1 : n_vars+n_greaterthan_cons+n_lessthan_cons] = Matrix(-I, n_greaterthan_cons, n_greaterthan_cons)

    # zero out the columns in V for upper level variables
    for col in upper_var_indices
        V[:,col.value] = spzeros(size(V,1), 1)
    end

    w = [b; d; f]
    return V, w, yu, yl, n_equality_cons, C, E
    # TODO use n_equality_cons to check rows from find_connected_rows_cols for values corresponding to constraints with slack variables
end
