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


function get_coef(var1::MOI.VariableIndex, var2::MOI.VariableIndex, sqts::Vector{MOI.ScalarQuadraticTerm{R}}) where R <: Real
    coef = 0
    for sqft in sqts
        if var1 == sqft.variable_index_1 && var2 == sqft.variable_index_2
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
	d = Inf*ones(nrows)
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

    given A[x:y] = b, C[x;y] ≤ d, E[x;y] ≥ f
    collect all of y bounds into 
    yl ≤ y ≤ yu
    and put all inequality constraints into equalities with slack variables s:
    Ux + V [y; s] = [b; d; f]

    NOTE: U and V are sparse arrays with columns for all variables in model s.t. that variable indices line up
"""
function standard_form(m; upper_var_indices=Vector{MOI.VariableIndex}())
	nvars = MOI.get(m, MOI.NumberOfVariables())
	con_types = MOI.get(m, MOI.ListOfConstraints())

    n_equality_cons = 0  # A[x;y] = b
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
	# TODO SingleVariable, EqualTo 
    # TODO what is best way to map variable indices in the standard form model to the LL model?

    #=
    For each ScalarAffineFunction ≥ or ≤ Float64 we add a slack variable and make the constraint
    an EqualTo{Float64}
    =#
    n_lessthan_cons = 0  # C[x;y] ≤ d
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
    yl = -Inf*ones(MOI.get(m, MOI.NumberOfVariables()))
	if (MOI.SingleVariable, MOI.GreaterThan{Float64}) in con_types

		singleVar_gt_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.SingleVariable, 
			MOI.GreaterThan{Float64}
		}());

		yl = BilevelJuMP.get_coef_matrix_and_rhs_vec(m, singleVar_gt_indices)
	end
	
    yu = Inf*ones(MOI.get(m, MOI.NumberOfVariables()))
	if (MOI.SingleVariable, MOI.LessThan{Float64}) in con_types

		singleVar_lt_indices = MOI.get(m, MOI.ListOfConstraintIndices{
			MOI.SingleVariable, 
			MOI.LessThan{Float64}
		}());

		yu = get_coef_matrix_and_rhs_vec(m, singleVar_lt_indices)
	end

    # remove rows from C that only apply to one variable by moving them to the bounds in yu
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

    # zero out the columns in V for upper level variables and build U
    U = spzeros(size(V,1), size(V,2)) # coefficients of UL variables
    for col in upper_var_indices
        U[:,col.value] = copy(V[:, col.value])
        V[:,col.value] = spzeros(size(V,1), 1)
    end

    w = [b; d; f]
    return U, V, w # , yu, yl, n_equality_cons, C, E
    # TODO use n_equality_cons to check rows from find_connected_rows_cols for values corresponding to constraints with slack variables
    # TODO build standard form model before Dualization?
end


"""

Construct the set A_N, which is the indices of lower level variables in the upper level 
objective of the form λ^T A y, where λ are the dual variables of lower level equality constraints and
y are lower level primal variables.

Also find the quadratic upper objective terms with lower primal * lower dual and return the maps for
the upper variable index (of the lower dual variable):
- to the lower primal term and 
- to the quadratic term in the UL objective

"""
function check_upper_objective_for_bilinear_linearization(upper, upper_to_lower_var_indices, upper_var_lower_ctr)
    A_N = Int[]
    upper_dual_to_quad_term = Dict{BilevelJuMP.VI, MOI.ScalarQuadraticTerm}()
    # upper_primal_to_lower_primal = Dict{BilevelJuMP.VI, BilevelJuMP.VI}()
    upper_dual_to_lower_primal = Dict{BilevelJuMP.VI, BilevelJuMP.VI}()
    lower_primal_var_to_lower_con = Dict{BilevelJuMP.VI, BilevelJuMP.CI}()

    UL_obj_type = MOI.get(upper, MOI.ObjectiveFunctionType())
    upper_obj_func_quad_terms = MOI.get(
        upper, MOI.ObjectiveFunction{UL_obj_type}()).quadratic_terms

    for upper_dual_var_idx in keys(upper_var_lower_ctr)

        for term in upper_obj_func_quad_terms

            if upper_dual_var_idx == term.variable_index_1
                if term.variable_index_2 in values(upper_to_lower_var_indices)
                    lower_primal_var_idx = upper_to_lower_var_indices[term.variable_index_2]
                    push!(A_N, lower_primal_var_idx.value)
                    # upper_primal_to_lower_primal[term.variable_index_2] = lower_primal_var_idx
                    upper_dual_to_quad_term[upper_dual_var_idx] = term  
                    upper_dual_to_lower_primal[upper_dual_var_idx] = lower_primal_var_idx
                    lower_primal_var_to_lower_con[lower_primal_var_idx] = upper_var_lower_ctr[upper_dual_var_idx]
                end

            elseif upper_dual_var_idx == term.variable_index_2
                if term.variable_index_1 in values(upper_to_lower_var_indices)
                    lower_primal_var_idx = upper_to_lower_var_indices[term.variable_index_1]
                    push!(A_N, lower_primal_var_idx.value)
                    # upper_primal_to_lower_primal[term.variable_index_1] = lower_primal_var_idx
                    upper_dual_to_quad_term[upper_dual_var_idx] = term
                    upper_dual_to_lower_primal[upper_dual_var_idx] = lower_primal_var_idx
                    lower_primal_var_to_lower_con[lower_primal_var_idx] = upper_var_lower_ctr[upper_dual_var_idx]
                end
            end
        end
    end

    return A_N, upper_dual_to_quad_term, upper_dual_to_lower_primal, lower_primal_var_to_lower_con
end


function get_lower_obj_coefs_of_upper_times_lower_primals(lower, lower_var_indices_of_upper_vars, A_N)
    AB_N = Int[]
    nvars = MOI.get(lower, MOI.NumberOfVariables())
    # B is in LL objective: x^T B y
    B = spzeros(nvars, nvars)

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
    if !isnothing(lower_obj_quad_terms)  # check for values in AB_N, 
        for term in lower_obj_quad_terms
            if term.variable_index_1 in lower_var_indices_of_upper_vars # UL var
                if term.variable_index_2.value in A_N  # LL var
                    push!(AB_N, term.variable_index_2.value)  # AB_N is not empty
                    B[term.variable_index_1.value, term.variable_index_2.value] = 
                        get_coef(term.variable_index_1, term.variable_index_2, lower_obj_quad_terms)
                end 
            elseif term.variable_index_2 in lower_var_indices_of_upper_vars  # UL var
                if term.variable_index_1.value in A_N  # LL var
                    push!(AB_N, term.variable_index_1.value) # AB_N is not empty
                    B[term.variable_index_1.value, term.variable_index_2.value] = 
                        get_coef(term.variable_index_1, term.variable_index_2, lower_obj_quad_terms)
                end 
            end
        end
    end
    return AB_N, B, lower_obj_terms, lower_obj_type_handled
end


function linear_terms_for_empty_AB(
        lower,
        upper_var_lower_ctr,
        bilinear_upper_dual_to_lower_primal,
        V,
        w,
        bilinear_upper_dual_to_quad_term,
        upper_to_m_idxmap,
        lower_obj_terms,
        lower_to_m_idxmap,
        lower_primal_dual_map,
        lower_dual_idxmap
    )
    linearizations = Vector{MOI.ScalarAffineTerm}()
    con_type = MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64}}

    for (upper_var, lower_con) in upper_var_lower_ctr
        j = lower_con.value
        n = bilinear_upper_dual_to_lower_primal[upper_var].value

        J_j, N_n = BilevelJuMP.find_connected_rows_cols(V, j, n, skip_1st_col_check=false)

        A_jn = bilinear_upper_dual_to_quad_term[upper_var].coefficient
        V_jn = V[j,n]
        
        for j_prime in J_j
            lower_con_index = con_type(j_prime)
            lower_dual_var = lower_primal_dual_map.primal_con_dual_var[lower_con_index][1]

            push!(linearizations,
                MOI.ScalarAffineTerm(A_jn / V_jn * w[j_prime], lower_dual_idxmap[lower_dual_var])  
            )
        end

        # TODO assert that lower level constraints in upper_var_lower_ctr are linear
        
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
            lo_bound_index = MOI.ConstraintIndex{MOI.SingleVariable, MOI.GreaterThan{Float64}}(lower_var.value)
            up_bound_index = MOI.ConstraintIndex{MOI.SingleVariable, MOI.LessThan{Float64}}(lower_var.value)
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

    return linearizations
end


function linear_terms_for_non_empty_AB(
        lower,
        upper_var_lower_ctr,
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
    linearizations = Vector{MOI.ScalarAffineTerm}()
    con_type = MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64}}

    for (upper_var, lower_con) in upper_var_lower_ctr
        j = lower_con.value
        n = bilinear_upper_dual_to_lower_primal[upper_var].value

        rows, cols = BilevelJuMP.find_connected_rows_cols(V, j, n, skip_1st_col_check=true)
        # rows is set J_j, cols is set N_n

        A_jn = bilinear_upper_dual_to_quad_term[upper_var].coefficient
        V_jn = V[j,n]
        p = A_jn / V_jn
        for r in rows
            lower_con_index = con_type(r)
            lower_dual_var = lower_primal_dual_map.primal_con_dual_var[lower_con_index][1]

            push!(linearizations,
                MOI.ScalarAffineTerm(p*w[r], lower_dual_idxmap[lower_dual_var])  
            )
        end

        # TODO assert that lower level constraints in upper_var_lower_ctr are linear
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
            lo_bound_index = MOI.ConstraintIndex{MOI.SingleVariable, MOI.GreaterThan{Float64}}(lower_var.value)
            up_bound_index = MOI.ConstraintIndex{MOI.SingleVariable, MOI.LessThan{Float64}}(lower_var.value)
            low_dual = get(lower_primal_dual_map.primal_con_dual_var, lo_bound_index, [nothing])[1]
            upp_dual = get(lower_primal_dual_map.primal_con_dual_var, up_bound_index, [nothing])[1]

            # have to use opposite signs of paper for these terms (b/c Dualization sets variable bound dual variables to be non-positive?)
            if low_bound != -Inf && !isnothing(low_dual)
                push!(linearizations, MOI.ScalarAffineTerm(-p * low_bound, lower_dual_idxmap[low_dual]))
            end
            if upp_bound != Inf && !isnothing(upp_dual) # TODO add a big number in place of Inf ?
                push!(linearizations, MOI.ScalarAffineTerm( p * upp_bound, lower_dual_idxmap[upp_dual]))
            end

        end
    end

    return linearizations
end