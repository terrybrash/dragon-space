// *****************************************************************************
// SmallList.hpp:
// *****************************************************************************
#ifndef SMALL_LIST_HPP
#define SMALL_LIST_HPP

#include <cstdlib>
#include <cstring>
#include <cassert>
#include <vector>

/// Stores a random-access sequence of elements similar to vector, but avoids 
/// heap allocations for small lists. T must be trivially constructible and 
/// destructible.
template <class T>
class SmallList
{
public:
    // Creates an empty list.
    SmallList();

    // Creates a copy of the specified list.
    SmallList(const SmallList& other);

    // Copies the specified list.
    SmallList& operator=(const SmallList& other);

    // Destroys the list.
    ~SmallList();

    // Returns the number of agents in the list.
    int size() const;

    // Returns the nth element.
    T& operator[](int n);

    // Returns the nth element in the list.
    const T& operator[](int n) const;

    // Returns an index to a matching element in the list or -1
    // if the element is not found.
    int find_index(const T& element) const;

    // Clears the list.
    void clear();

    // Reserves space for n elements.
    void reserve(int n);

    // Inserts an element to the back of the list.
    void push_back(const T& element);

    /// Pops an element off the back of the list.
    T pop_back();

    // Swaps the contents of this list with the other.
    void swap(SmallList& other);

    // Returns a pointer to the underlying buffer.
    T* data();

    // Returns a pointer to the underlying buffer.
    const T* data() const;

private:
    enum {fixed_cap = 256};
    struct ListData
    {
        ListData();
        T buf[fixed_cap];
        T* data;
        int num;
        int cap;
    };
    ListData ld;
};

/// Provides an indexed free list with constant-time removals from anywhere
/// in the list without invalidating indices. T must be trivially constructible 
/// and destructible.
template <class T>
class FreeList
{
public:
    /// Creates a new free list.
    FreeList();

    /// Inserts an element to the free list and returns an index to it.
    int insert(const T& element);

    // Removes the nth element from the free list.
    void erase(int n);

    // Removes all elements from the free list.
    void clear();

    // Returns the range of valid indices.
    int range() const;

    // Returns the nth element.
    T& operator[](int n);

    // Returns the nth element.
    const T& operator[](int n) const;

    // Reserves space for n elements.
    void reserve(int n);

    // Swaps the contents of the two lists.
    void swap(FreeList& other);

private:
    union FreeElement
    {
        T element;
        int next;
    };
    SmallList<FreeElement> data;
    int first_free;
};

// ---------------------------------------------------------------------------------
// SmallList Implementation
// ---------------------------------------------------------------------------------
template <class T>
SmallList<T>::ListData::ListData(): data(buf), num(0), cap(fixed_cap)
{
}

template <class T>
SmallList<T>::SmallList()
{
}

template <class T>
SmallList<T>::SmallList(const SmallList& other)
{
    if (other.ld.cap == fixed_cap)
    {
        ld = other.ld;
        ld.data = ld.buf;
    }
    else
    {
        reserve(other.ld.num);
        for (int j=0; j < other.size(); ++j)
            ld.data[j] = other.ld.data[j];
        ld.num = other.ld.num;
        ld.cap = other.ld.cap;
    }
}

template <class T>
SmallList<T>& SmallList<T>::operator=(const SmallList<T>& other)
{
    SmallList(other).swap(*this);
    return *this;
}

template <class T>
SmallList<T>::~SmallList()
{
    if (ld.data != ld.buf)
        free(ld.data);
}

template <class T>
int SmallList<T>::size() const
{
    return ld.num;
}

template <class T>
T& SmallList<T>::operator[](int n)
{
    assert(n >= 0 && n < ld.num);
    return ld.data[n];
}

template <class T>
const T& SmallList<T>::operator[](int n) const
{
    assert(n >= 0 && n < ld.num);
    return ld.data[n];
}

template <class T>
int SmallList<T>::find_index(const T& element) const
{
    for (int j=0; j < ld.num; ++j)
    {
        if (ld.data[j] == element)
            return j;
    }
    return -1;
}

template <class T>
void SmallList<T>::clear()
{
    ld.num = 0;
}

template <class T>
void SmallList<T>::reserve(int n)
{
    enum {type_size = sizeof(T)};
    if (n > ld.cap)
    {
        if (ld.cap == fixed_cap)
        {
            ld.data = static_cast<T*>(malloc(n * type_size));
            memcpy(ld.data, ld.buf, sizeof(ld.buf));
        }
        else
            ld.data = static_cast<T*>(realloc(ld.data, n * type_size));
        ld.cap = n;
    }
}

template <class T>
void SmallList<T>::push_back(const T& element)
{
    if (ld.num >= ld.cap)
        reserve(ld.cap * 2);
    ld.data[ld.num++] = element;
}

template <class T>
T SmallList<T>::pop_back()
{
    return ld.data[--ld.num];
}

template <class T>
void SmallList<T>::swap(SmallList& other)
{
    ListData& ld1 = ld;
    ListData& ld2 = other.ld;

    const int use_fixed1 = ld1.data == ld1.buf;
    const int use_fixed2 = ld2.data == ld2.buf;

    const ListData temp = ld1;
    ld1 = ld2;
    ld2 = temp;

    if (use_fixed1)
        ld2.data = ld2.buf;
    if (use_fixed2)
        ld1.data = ld1.buf;
}

template <class T>
T* SmallList<T>::data()
{
    return ld.data;
}

template <class T>
const T* SmallList<T>::data() const
{
    return ld.data;
}

// ---------------------------------------------------------------------------------
// FreeList Implementation
// ---------------------------------------------------------------------------------
template <class T>
FreeList<T>::FreeList(): first_free(-1)
{
}

template <class T>
int FreeList<T>::insert(const T& element)
{
    if (first_free != -1)
    {
        const int index = first_free;
        first_free = data[first_free].next;
        data[index].element = element;
        return index;
    }
    else
    {
        FreeElement fe;
        fe.element = element;
        data.push_back(fe);
        return data.size() - 1;
    }
}

template <class T>
void FreeList<T>::erase(int n)
{
    assert(n >= 0 && n < data.size());
    data[n].next = first_free;
    first_free = n;
}

template <class T>
void FreeList<T>::clear()
{
    data.clear();
    first_free = -1;
}

template <class T>
int FreeList<T>::range() const
{
    return data.size();
}

template <class T>
T& FreeList<T>::operator[](int n)
{
    return data[n].element;
}

template <class T>
const T& FreeList<T>::operator[](int n) const
{
    return data[n].element;
}

template <class T>
void FreeList<T>::reserve(int n)
{
    data.reserve(n);
}

template <class T>
void FreeList<T>::swap(FreeList& other)
{
    const int temp = first_free;
    data.swap(other.data);
    first_free = other.first_free;
    other.first_free = temp;
}

#endif

// *****************************************************************************
// UGrid.hpp
// *****************************************************************************
#ifndef UGRID_HPP
#define UGRID_HPP

#include "SmallList.hpp"

struct UGridElt
{
    // Stores the next element in the cell.
    int next;

    // Stores the ID of the element. This can be used to associate external
    // data to the element.
    int id;

    // Stores the center position of the uniformly-sized element.
    float mx, my;
};

struct UGridRow
{
    // Stores all the elements in the grid row.
    FreeList<UGridElt> elts;

    // Stores all the cells in the row. Each cell stores an index pointing to 
    // the first element in that cell, or -1 if the cell is empty.
    int* cells;

    // Stores the number of elements in the row.
    int num_elts;
};

struct UGrid
{
    // Stores all the rows in the grid.
    UGridRow* rows;

    // Stores the number of columns, rows, and cells in the grid.
    int num_cols, num_rows, num_cells;

    // Stores the inverse size of a cell.
    float inv_cell_w, inv_cell_h;

    // Stores the half-size of all elements stored in the grid.
    float hx, hy;

    // Stores the upper-left corner of the grid.
    float x, y;

    // Stores the size of the grid.
    float w, h;
};

// Returns a new grid storing elements that have a uniform upper-bound size. Because 
// all elements are treated uniformly-sized for the sake of search queries, each one 
// can be stored as a single point in the grid.
UGrid* ugrid_create(float hx, float hy, float cell_w, float cell_h, 
                    float l, float t, float r, float b);

// Destroys the grid.
void ugrid_destroy(UGrid* grid);

// Returns the grid cell X index for the specified position.
int ugrid_cell_x(const UGrid* grid, float x);

// Returns the grid cell Y index for the specified position.
int ugrid_cell_y(const UGrid* grid, float y);

// Inserts an element to the grid.
void ugrid_insert(UGrid* grid, int id, float mx, float my);

// Removes an element from the grid.
void ugrid_remove(UGrid* grid, int id, float mx, float my);

// Moves an element in the grid from the former position to the new one.
void ugrid_move(UGrid* grid, int id, float prev_mx, float prev_my, float mx, float my);

// Returns all the element IDs that intersect the specified rectangle excluding 
// elements with the specified ID to omit.
SmallList<int> ugrid_query(const UGrid* grid, float mx, float my, float hx, float hy, int omit_id);

// Returns true if the specified element position is inside the grid boundaries.
bool ugrid_in_bounds(const UGrid* grid, float mx, float my);

// Optimizes the grid, rearranging the memory of the grid to allow cache-friendly 
// cell traversal.
void ugrid_optimize(UGrid* grid);

#endif

// *****************************************************************************
// UGrid.cpp
// *****************************************************************************
#include "UGrid.hpp"
#include <cmath>

static int ceil_div(float value, float divisor)
{
    // Returns the value divided by the divisor rounded up.
    const float resultf = value / divisor;
    const int result = (int)resultf;
    return result < resultf ? result+1: result;
}

static int min_int(int a, int b)
{
    assert(sizeof(int) == 4);
    a -= b;
    a &= a >> 31;
    return a + b;
}

static int max_int(int a, int b)
{
    assert(sizeof(int) == 4);
    a -= b;
    a &= (~a) >> 31;
    return a + b;
}

static int to_cell_idx(float val, float inv_cell_size, int num_cells)
{
    const int cell_pos = (int)(val * inv_cell_size);
    return min_int(max_int(cell_pos, 0), num_cells - 1);
}

UGrid* ugrid_create(float hx, float hy, float cell_w, float cell_h, 
                    float l, float t, float r, float b)
{
    const float w = r - l, h = b - t;
    const int num_cols = ceil_div(w, cell_w), num_rows = ceil_div(h, cell_h);

    UGrid* grid = new UGrid;
    grid->num_cols = num_cols;
    grid->num_rows = num_rows;
    grid->num_cells = num_cols * num_rows;
    grid->inv_cell_w = 1.0f / cell_w;
    grid->inv_cell_h = 1.0f / cell_w;
    grid->x = l;
    grid->y = t;
    grid->h = w;
    grid->w = h;
    grid->hx = hx;
    grid->hy = hy;

    grid->rows = new UGridRow[num_rows];
    for (int r=0; r < num_rows; ++r)
    {
        grid->rows[r].cells = new int[num_cols];
        for (int c=0; c < num_cols; ++c)
            grid->rows[r].cells[c] = -1;
    }
    return grid;
}

void ugrid_destroy(UGrid* grid)
{
    for (int r=0; r < grid->num_rows; ++r)
        delete[] grid->rows[r].cells;
    delete[] grid->rows;
    delete grid;
}

int ugrid_cell_x(const UGrid* grid, float x)
{
    return to_cell_idx(x - grid->x, grid->inv_cell_w, grid->num_cols);
}

int ugrid_cell_y(const UGrid* grid, float y)
{
    return to_cell_idx(y - grid->y, grid->inv_cell_h, grid->num_rows);
}

void ugrid_insert(UGrid* grid, int id, float mx, float my)
{
    const int cell_x = ugrid_cell_x(grid, mx);
    const int cell_y = ugrid_cell_y(grid, my);
    UGridRow* row = &grid->rows[cell_y];
    int* cell = &row->cells[cell_x];

    const UGridElt new_elt = {*cell, id, mx, my};
    *cell = row->elts.insert(new_elt);
}

void ugrid_remove(UGrid* grid, int id, float mx, float my)
{
    const int cell_x = ugrid_cell_x(grid, mx);
    const int cell_y = ugrid_cell_y(grid, my);
    UGridRow* row = &grid->rows[cell_y];

    int* link = &row->cells[cell_x];
    while (row->elts[*link].id != id)
        link = &row->elts[*link].next;

    const int idx = *link;
    *link = row->elts[idx].next;
    row->elts.erase(idx);
}

void ugrid_move(UGrid* grid, int id, float prev_mx, float prev_my, float mx, float my)
{
    const int prev_cell_x = ugrid_cell_x(grid, prev_mx);
    const int prev_cell_y = ugrid_cell_y(grid, prev_my);
    const int next_cell_x = ugrid_cell_x(grid, mx);
    const int next_cell_y = ugrid_cell_y(grid, my);
    UGridRow* prev_row = &grid->rows[prev_cell_y];

    if (next_cell_x == prev_cell_y && next_cell_x == next_cell_y)
    {
        // If the element will still belong in the same cell, simply update its position.
        int elt_idx = prev_row->cells[prev_cell_x];
        while (prev_row->elts[elt_idx].id != id)
            elt_idx = prev_row->elts[elt_idx].next;

        // Update the element's position.
        prev_row->elts[elt_idx].mx = mx;
        prev_row->elts[elt_idx].my = my;
    }
    else
    {
        // Otherwise if the element will move to another cell, remove it first from the
        // previous cell and insert it to the new one.
        UGridRow* next_row = &grid->rows[next_cell_y];
        int* link = &prev_row->cells[prev_cell_x];
        while (prev_row->elts[*link].id != id)
            link = &prev_row->elts[*link].next;

        // Remove the element from the previous cell and row.
        const int elt_idx = *link;
        UGridElt elt = prev_row->elts[elt_idx];
        *link = prev_row->elts[elt_idx].next;
        prev_row->elts.erase(elt_idx);

        // Update the element's position.
        prev_row->elts[elt_idx].mx = mx;
        prev_row->elts[elt_idx].my = my;

        // Insert it to the new row and cell.
        elt.next = next_row->cells[next_cell_x];
        next_row->cells[next_cell_x] = next_row->elts.insert(elt);
    }
}

SmallList<int> ugrid_query(const UGrid* grid, float mx, float my, float hx, float hy, int omit_id)
{
    // Expand the size of the query by the upper-bound uniform size of the elements. This 
    // expansion is what allows us to find elements based only on their point.
    const float fx = hx + grid->hx;
    const float fy = hy + grid->hy;

    // Find the cells that intersect the search query.
    const int min_x = ugrid_cell_x(grid, mx - fx);
    const int min_y = ugrid_cell_y(grid, my - fy);
    const int max_x = ugrid_cell_x(grid, mx + fx);
    const int max_y = ugrid_cell_y(grid, my + fy);

    // Find the elements that intersect the search query.
    SmallList<int> res;
    for (int y = min_y; y <= max_y; ++y)
    {
        const UGridRow* row = &grid->rows[y];
        for (int x = min_x; x <= max_x; ++x)
        {
            int elt_idx = row->cells[x];
            while (elt_idx != -1)
            {
                const UGridElt* elt = &row->elts[elt_idx];
                if (elt_idx != omit_id && fabs(mx - elt->mx) <= fx && fabs(my - elt->my) <= fy)
                    res.push_back(elt_idx);
                elt_idx = row->elts[elt_idx].next;
            }
        }
    }
    return res;
}

bool ugrid_in_bounds(const UGrid* grid, float mx, float my)
{
    mx -= grid->x;
    my -= grid->y;
    const float x1 = mx-grid->hx, y1 = my-grid->hy;
    const float x2 = mx+grid->hx, y2 = my+grid->hy;
    return x1 >= 0.0f && x2 < grid->w && y1 >= 0.0f && y2 < grid->h;
}

void ugrid_optimize(UGrid* grid)
{
    for (int r=0; r < grid->num_rows; ++r)
    {
        FreeList<UGridElt> new_elts;
        UGridRow* row = &grid->rows[r];
        new_elts.reserve(row->num_elts);
        for (int c=0; c < grid->num_cols; ++c)
        {
            // Replace links to the old elements list to links in the new
            // cache-friendly element list.
            SmallList<int> new_elt_idxs;
            int* link = &row->cells[c];
            while (*link != -1)
            {
                const UGridElt* elt = &row->elts[*link];
                new_elt_idxs.push_back(new_elts.insert(*elt));
                *link = elt->next;
            }
            for (int j=0; j < new_elt_idxs.size(); ++j)
            {
                const int new_elt_idx = new_elt_idxs[j];
                new_elts[new_elt_idx].next = *link;
                *link = new_elt_idx;
            }
        }
        // Swap the new element list with the old one.
        row->elts.swap(new_elts);
    }
}