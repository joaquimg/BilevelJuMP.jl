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
    find_connected_rows_cols(A::AbstractArray, row::Int, col::Int)
    
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

