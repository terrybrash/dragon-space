#ifndef INT_LIST_H
#define INT_LIST_H

#ifdef __cplusplus
    #define IL_FUNC extern "C"
#else
    #define IL_FUNC
#endif

typedef struct IntList IntList;
enum {il_fixed_cap = 128};

struct IntList
{
    // Stores a fixed-size buffer in advance to avoid requiring
    // a heap allocation until we run out of space.
    int fixed[il_fixed_cap];

    // Points to the buffer used by the list. Initially this will
    // point to 'fixed'.
    int* data;

    // Stores how many integer fields each element has.
    int num_fields;

    // Stores the number of elements in the list.
    int num;

    // Stores the capacity of the array.
    int cap;

    // Stores an index to the free element or -1 if the free list
    // is empty.
    int free_element;
};

// ---------------------------------------------------------------------------------
// List Interface
// ---------------------------------------------------------------------------------
// Creates a new list of elements which each consist of integer fields.
// 'num_fields' specifies the number of integer fields each element has.
IL_FUNC void il_create(IntList* il, int num_fields);

// Destroys the specified list.
IL_FUNC void il_destroy(IntList* il);

// Returns the number of elements in the list.
IL_FUNC int il_size(const IntList* il);

// Returns the value of the specified field for the nth element.
IL_FUNC int il_get(const IntList* il, int n, int field);

// Sets the value of the specified field for the nth element.
IL_FUNC void il_set(IntList* il, int n, int field, int val);

// Clears the specified list, making it empty.
IL_FUNC void il_clear(IntList* il);

// ---------------------------------------------------------------------------------
// Stack Interface (do not mix with free list usage; use one or the other)
// ---------------------------------------------------------------------------------
// Inserts an element to the back of the list and returns an index to it.
IL_FUNC int il_push_back(IntList* il);

// Removes the element at the back of the list.
IL_FUNC void il_pop_back(IntList* il);

// ---------------------------------------------------------------------------------
// Free List Interface (do not mix with stack usage; use one or the other)
// ---------------------------------------------------------------------------------
// Inserts an element to a vacant position in the list and returns an index to it.
IL_FUNC int il_insert(IntList* il);

// Removes the nth element in the list.
IL_FUNC void il_erase(IntList* il, int n);

#endif

#include "IntList.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>

void il_create(IntList* il, int num_fields)
{
    il->data = il->fixed;
    il->num = 0;
    il->cap = il_fixed_cap;
    il->num_fields = num_fields;
    il->free_element = -1;
}

void il_destroy(IntList* il)
{
    // Free the buffer only if it was heap allocated.
    if (il->data != il->fixed)
        free(il->data);
}

void il_clear(IntList* il)
{
    il->num = 0;
    il->free_element = -1;
}

int il_size(const IntList* il)
{
    return il->num;
}

int il_get(const IntList* il, int n, int field)
{
    assert(n >= 0 && n < il->num);
    return il->data[n*il->num_fields + field];
}

void il_set(IntList* il, int n, int field, int val)
{
    assert(n >= 0 && n < il->num);
    il->data[n*il->num_fields + field] = val;
}

int il_push_back(IntList* il)
{
    const int new_pos = (il->num+1) * il->num_fields;

    // If the list is full, we need to reallocate the buffer to make room
    // for the new element.
    if (new_pos > il->cap)
    {
        // Use double the size for the new capacity.
        const int new_cap = new_pos * 2;

        // If we're pointing to the fixed buffer, allocate a new array on the
        // heap and copy the fixed buffer contents to it.
        if (il->cap == il_fixed_cap)
        {
            il->data = malloc(new_cap * sizeof(*il->data));
            memcpy(il->data, il->fixed, sizeof(il->fixed));
        }
        else
        {
            // Otherwise reallocate the heap buffer to the new size.
            il->data = realloc(il->data, new_cap * sizeof(*il->data));
        }
        // Set the old capacity to the new capacity.
        il->cap = new_cap;
    }
    return il->num++;
}

void il_pop_back(IntList* il)
{
    // Just decrement the list size.
    assert(il->num > 0);
    --il->num;
}

int il_insert(IntList* il)
{
    // If there's a free index in the free list, pop that and use it.
    if (il->free_element != -1)
    {
        const int index = il->free_element;
        const int pos = index * il->num_fields;

        // Set the free index to the next free index.
        il->free_element = il->data[pos];

        // Return the free index.
        return index;
    }
    // Otherwise insert to the back of the array.
    return il_push_back(il);
}

void il_erase(IntList* il, int n)
{
    // Push the element to the free list.
    const int pos = n * il->num_fields;
    il->data[pos] = il->free_element;
    il->free_element = n;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#ifndef QUADTREE_H
#define QUADTREE_H

#include "IntList.h"

#ifdef __cplusplus
    #define QTREE_FUNC extern "C"
#else
    #define QTREE_FUNC
#endif

typedef struct Quadtree Quadtree;

struct Quadtree
{
    // Stores all the nodes in the quadtree. The first node in this
    // sequence is always the root.
    IntList nodes;

    // Stores all the elements in the quadtree.
    IntList elts;

    // Stores all the element nodes in the quadtree.
    IntList enodes;

    // Stores the quadtree extents.
    int root_mx, root_my, root_sx, root_sy;

    // Maximum allowed elements in a leaf before the leaf is subdivided/split unless
    // the leaf is at the maximum allowed tree depth.
    int max_elements;

    // Stores the maximum depth allowed for the quadtree.
    int max_depth;

    // Temporary buffer used for queries.
    char* temp;

    // Stores the size of the temporary buffer.
    int temp_size;
};

// Function signature used for traversing a tree node.
typedef void QtNodeFunc(Quadtree* qt, void* user_data, int node, int depth, int mx, int my, int sx, int sy);

// Creates a quadtree with the requested extents, maximum elements per leaf, and maximum tree depth.
QTREE_FUNC void qt_create(Quadtree* qt, int width, int height, int max_elements, int max_depth);

// Destroys the quadtree.
QTREE_FUNC void qt_destroy(Quadtree* qt);

// Inserts a new element to the tree.
// Returns an index to the new element.
QTREE_FUNC int qt_insert(Quadtree* qt, int id, float x1, float y1, float x2, float y2);

// Removes the specified element from the tree.
QTREE_FUNC void qt_remove(Quadtree* qt, int element);

// Cleans up the tree, removing empty leaves.
QTREE_FUNC void qt_cleanup(Quadtree* qt);

// Outputs a list of elements found in the specified rectangle.
QTREE_FUNC void qt_query(Quadtree* qt, IntList* out, float x1, float y1, float x2, float y2, int omit_element);

// Traverses all the nodes in the tree, calling 'branch' for branch nodes and 'leaf'
// for leaf nodes.
QTREE_FUNC void qt_traverse(Quadtree* qt, void* user_data, QtNodeFunc* branch, QtNodeFunc* leaf);

#endif

#include "Quadtree.h"
#include <stdlib.h>

enum
{
    // ----------------------------------------------------------------------------------------
    // Element node fields:
    // ----------------------------------------------------------------------------------------
    enode_num = 2,

    // Points to the next element in the leaf node. A value of -1
    // indicates the end of the list.
    enode_idx_next = 0,

    // Stores the element index.
    enode_idx_elt = 1,

    // ----------------------------------------------------------------------------------------
    // Element fields:
    // ----------------------------------------------------------------------------------------
    elt_num = 5,

    // Stores the rectangle encompassing the element.
    elt_idx_lft = 0, elt_idx_top = 1, elt_idx_rgt = 2, elt_idx_btm = 3,

    // Stores the ID of the element.
    elt_idx_id = 4,

    // ----------------------------------------------------------------------------------------
    // Node fields:
    // ----------------------------------------------------------------------------------------
    node_num = 2,

    // Points to the first child if this node is a branch or the first element
    // if this node is a leaf.
    node_idx_fc = 0,

    // Stores the number of elements in the node or -1 if it is not a leaf.
    node_idx_num = 1,

    // ----------------------------------------------------------------------------------------
    // Node data fields:
    // ----------------------------------------------------------------------------------------
    nd_num = 6,

    // Stores the extents of the node using a centered rectangle and half-size.
    nd_idx_mx = 0, nd_idx_my = 1, nd_idx_sx = 2, nd_idx_sy = 3,

    // Stores the index of the node.
    nd_idx_index = 4,

    // Stores the depth of the node.
    nd_idx_depth = 5,
};

static void node_insert(Quadtree* qt, int index, int depth, int mx, int my, int sx, int sy, int element);

static int floor_int(float val)
{
    return (int)val;
}

static int intersect(int l1, int t1, int r1, int b1,
                     int l2, int t2, int r2, int b2)
{
    return l2 <= r1 && r2 >= l1 && t2 <= b1 && b2 >= t1;
}

void leaf_insert(Quadtree* qt, int node, int depth, int mx, int my, int sx, int sy, int element)
{
    // Insert the element node to the leaf.
    const int nd_fc = il_get(&qt->nodes, node, node_idx_fc);
    il_set(&qt->nodes, node, node_idx_fc, il_insert(&qt->enodes));
    il_set(&qt->enodes, il_get(&qt->nodes, node, node_idx_fc), enode_idx_next, nd_fc);
    il_set(&qt->enodes, il_get(&qt->nodes, node, node_idx_fc), enode_idx_elt, element);

    // If the leaf is full, split it.
    if (il_get(&qt->nodes, node, node_idx_num) == qt->max_elements && depth < qt->max_depth)
    {
        int fc = 0, j = 0;
        IntList elts = {0};
        il_create(&elts, 1);

        // Transfer elements from the leaf node to a list of elements.
        while (il_get(&qt->nodes, node, node_idx_fc) != -1)
        {
            const int index = il_get(&qt->nodes, node, node_idx_fc);
            const int next_index = il_get(&qt->enodes, index, enode_idx_next);
            const int elt = il_get(&qt->enodes, index, enode_idx_elt);

            // Pop off the element node from the leaf and remove it from the qt.
            il_set(&qt->nodes, node, node_idx_fc, next_index);
            il_erase(&qt->enodes, index);

            // Insert element to the list.
            il_set(&elts, il_push_back(&elts), 0, elt);
        }

        // Start by allocating 4 child nodes.
        fc = il_insert(&qt->nodes);
        il_insert(&qt->nodes);
        il_insert(&qt->nodes);
        il_insert(&qt->nodes);
        il_set(&qt->nodes, node, node_idx_fc, fc);

        // Initialize the new child nodes.
        for (j=0; j < 4; ++j)
        {
            il_set(&qt->nodes, fc+j, node_idx_fc, -1);
            il_set(&qt->nodes, fc+j, node_idx_num, 0);
        }

        // Transfer the elements in the former leaf node to its new children.
        il_set(&qt->nodes, node, node_idx_num, -1);
        for (j=0; j < il_size(&elts); ++j)
            node_insert(qt, node, depth, mx, my, sx, sy, il_get(&elts, j, 0));
        il_destroy(&elts);
    }
    else
    {
        // Increment the leaf element count.
        il_set(&qt->nodes, node, node_idx_num, il_get(&qt->nodes, node, node_idx_num) + 1);
    }
}

static void push_node(IntList* nodes, int nd_index, int nd_depth, int nd_mx, int nd_my, int nd_sx, int nd_sy)
{
    const int back_idx = il_push_back(nodes);
    il_set(nodes, back_idx, nd_idx_mx, nd_mx);
    il_set(nodes, back_idx, nd_idx_my, nd_my);
    il_set(nodes, back_idx, nd_idx_sx, nd_sx);
    il_set(nodes, back_idx, nd_idx_sy, nd_sy);
    il_set(nodes, back_idx, nd_idx_index, nd_index);
    il_set(nodes, back_idx, nd_idx_depth, nd_depth);
}

static void find_leaves(IntList* out, const Quadtree* qt, int node, int depth,
                        int mx, int my, int sx, int sy,
                        int lft, int top, int rgt, int btm)
{
    IntList to_process = {0};
    il_create(&to_process, nd_num);
    push_node(&to_process, node, depth, mx, my, sx, sy);

    while (il_size(&to_process) > 0)
    {
        const int back_idx = il_size(&to_process) - 1;
        const int nd_mx = il_get(&to_process, back_idx, nd_idx_mx);
        const int nd_my = il_get(&to_process, back_idx, nd_idx_my);
        const int nd_sx = il_get(&to_process, back_idx, nd_idx_sx);
        const int nd_sy = il_get(&to_process, back_idx, nd_idx_sy);
        const int nd_index = il_get(&to_process, back_idx, nd_idx_index);
        const int nd_depth = il_get(&to_process, back_idx, nd_idx_depth);
        il_pop_back(&to_process);

        // If this node is a leaf, insert it to the list.
        if (il_get(&qt->nodes, nd_index, node_idx_num) != -1)
            push_node(out, nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy);
        else
        {
            // Otherwise push the children that intersect the rectangle.
            const int fc = il_get(&qt->nodes, nd_index, node_idx_fc);
            const int hx = nd_sx >> 1, hy = nd_sy >> 1;
            const int l = nd_mx-hx, t = nd_my-hx, r = nd_mx+hx, b = nd_my+hy;

            if (top <= nd_my)
            {
                if (lft <= nd_mx)
                    push_node(&to_process, fc+0, nd_depth+1, l,t,hx,hy);
                if (rgt > nd_mx)
                    push_node(&to_process, fc+1, nd_depth+1, r,t,hx,hy);
            }
            if (btm > nd_my)
            {
                if (lft <= nd_mx)
                    push_node(&to_process, fc+2, nd_depth+1, l,b,hx,hy);
                if (rgt > nd_mx)
                    push_node(&to_process, fc+3, nd_depth+1, r,b,hx,hy);
            }
        }
    }
    il_destroy(&to_process);
}

static void node_insert(Quadtree* qt, int index, int depth, int mx, int my, int sx, int sy, int element)
{
    // Find the leaves and insert the element to all the leaves found.
    int j = 0;
    IntList leaves = {0};

    const int lft = il_get(&qt->elts, element, elt_idx_lft);
    const int top = il_get(&qt->elts, element, elt_idx_top);
    const int rgt = il_get(&qt->elts, element, elt_idx_rgt);
    const int btm = il_get(&qt->elts, element, elt_idx_btm);

    il_create(&leaves, nd_num);
    find_leaves(&leaves, qt, index, depth, mx, my, sx, sy, lft, top, rgt, btm);
    for (j=0; j < il_size(&leaves); ++j)
    {
        const int nd_mx = il_get(&leaves, j, nd_idx_mx);
        const int nd_my = il_get(&leaves, j, nd_idx_my);
        const int nd_sx = il_get(&leaves, j, nd_idx_sx);
        const int nd_sy = il_get(&leaves, j, nd_idx_sy);
        const int nd_index = il_get(&leaves, j, nd_idx_index);
        const int nd_depth = il_get(&leaves, j, nd_idx_depth);
        leaf_insert(qt, nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy, element);
    }
    il_destroy(&leaves);
}

void qt_create(Quadtree* qt, int width, int height, int max_elements, int max_depth)
{
    qt->max_elements = max_elements;
    qt->max_depth = max_depth;
    qt->temp = 0;
    qt->temp_size = 0;
    il_create(&qt->nodes, node_num);
    il_create(&qt->elts, elt_num);
    il_create(&qt->enodes, enode_num);

    // Insert the root node to the qt.
    il_insert(&qt->nodes);
    il_set(&qt->nodes, 0, node_idx_fc, -1);
    il_set(&qt->nodes, 0, node_idx_num, 0);

    // Set the extents of the root node.
    qt->root_mx = width >> 1;
    qt->root_my = height >> 1;
    qt->root_sx = qt->root_mx;
    qt->root_sy = qt->root_my;
}

void qt_destroy(Quadtree* qt)
{
    il_destroy(&qt->nodes);
    il_destroy(&qt->elts);
    il_destroy(&qt->enodes);
    free(qt->temp);
}

int qt_insert(Quadtree* qt, int id, float x1, float y1, float x2, float y2)
{
    // Insert a new element.
    const int new_element = il_insert(&qt->elts);

    // Set the fields of the new element.
    il_set(&qt->elts, new_element, elt_idx_lft, floor_int(x1));
    il_set(&qt->elts, new_element, elt_idx_top, floor_int(y1));
    il_set(&qt->elts, new_element, elt_idx_rgt, floor_int(x2));
    il_set(&qt->elts, new_element, elt_idx_btm, floor_int(y2));
    il_set(&qt->elts, new_element, elt_idx_id, id);

    // Insert the element to the appropriate leaf node(s).
    node_insert(qt, 0, 0, qt->root_mx, qt->root_my, qt->root_sx, qt->root_sy, new_element);
    return new_element;
}

void qt_remove(Quadtree* qt, int element)
{
    // Find the leaves.
    int j = 0;
    IntList leaves = {0};

    const int lft = il_get(&qt->elts, element, elt_idx_lft);
    const int top = il_get(&qt->elts, element, elt_idx_top);
    const int rgt = il_get(&qt->elts, element, elt_idx_rgt);
    const int btm = il_get(&qt->elts, element, elt_idx_btm);

    il_create(&leaves, nd_num);
    find_leaves(&leaves, qt, 0, 0, qt->root_mx, qt->root_my, qt->root_sx, qt->root_sy, lft, top, rgt, btm);

    // For each leaf node, remove the element node.
    for (j=0; j < il_size(&leaves); ++j)
    {
        const int nd_index = il_get(&leaves, j, nd_idx_index);

        // Walk the list until we find the element node.
        int node_index = il_get(&qt->nodes, nd_index, node_idx_fc);
        int prev_index = -1;
        while (node_index != -1 && il_get(&qt->enodes, node_index, enode_idx_elt) != element)
        {
            prev_index = node_index;
            node_index = il_get(&qt->enodes, node_index, enode_idx_next);
        }

        if (node_index != -1)
        {
            // Remove the element node.
            const int next_index = il_get(&qt->enodes, node_index, enode_idx_next);
            if (prev_index == -1)
                il_set(&qt->nodes, nd_index, node_idx_fc, next_index);
            else
                il_set(&qt->enodes, prev_index, enode_idx_next, next_index);
            il_erase(&qt->enodes, node_index);

            // Decrement the leaf element count.
            il_set(&qt->nodes, nd_index, node_idx_num, il_get(&qt->nodes, nd_index, node_idx_num)-1);
        }
    }
    il_destroy(&leaves);

    // Remove the element.
    il_erase(&qt->elts, element);
}

void qt_query(Quadtree* qt, IntList* out, float x1, float y1, float x2, float y2, int omit_element)
{
    // Find the leaves that intersect the specified query rectangle.
    int j = 0;
    IntList leaves = {0};
    const int elt_cap = il_size(&qt->elts);

    const int qlft = floor_int(x1);
    const int qtop = floor_int(y1);
    const int qrgt = floor_int(x2);
    const int qbtm = floor_int(y2);

    if (qt->temp_size < elt_cap)
    {
        qt->temp_size = elt_cap;
        qt->temp = realloc(qt->temp, qt->temp_size * sizeof(*qt->temp));
        memset(qt->temp, 0, qt->temp_size * sizeof(*qt->temp));
    }

    // For each leaf node, look for elements that intersect.
    il_create(&leaves, nd_num);
    find_leaves(&leaves, qt, 0, 0, qt->root_mx, qt->root_my, qt->root_sx, qt->root_sy, qlft, qtop, qrgt, qbtm);

    il_clear(out);
    for (j=0; j < il_size(&leaves); ++j)
    {
        const int nd_index = il_get(&leaves, j, nd_idx_index);

        // Walk the list and add elements that intersect.
        int elt_node_index = il_get(&qt->nodes, nd_index, node_idx_fc);
        while (elt_node_index != -1)
        {
            const int element = il_get(&qt->enodes, elt_node_index, enode_idx_elt);
            const int lft = il_get(&qt->elts, element, elt_idx_lft);
            const int top = il_get(&qt->elts, element, elt_idx_top);
            const int rgt = il_get(&qt->elts, element, elt_idx_rgt);
            const int btm = il_get(&qt->elts, element, elt_idx_btm);
            if (!qt->temp[element] && element != omit_element && intersect(qlft,qtop,qrgt,qbtm, lft,top,rgt,btm))
            {
                il_set(out, il_push_back(out), 0, element);
                qt->temp[element] = 1;
            }
            elt_node_index = il_get(&qt->enodes, elt_node_index, enode_idx_next);
        }
    }
    il_destroy(&leaves);

    // Unmark the elements that were inserted.
    for (j=0; j < il_size(out); ++j)
        qt->temp[il_get(out, j, 0)] = 0;
}

void qt_cleanup(Quadtree* qt)
{
    IntList to_process = {0};
    il_create(&to_process, 1);

    // Only process the root if it's not a leaf.
    if (il_get(&qt->nodes, 0, node_idx_num) == -1)
    {
        // Push the root index to the stack.
        il_set(&to_process, il_push_back(&to_process), 0, 0);
    }

    while (il_size(&to_process) > 0)
    {
        // Pop a node from the stack.
        const int node = il_get(&to_process, il_size(&to_process)-1, 0);
        const int fc = il_get(&qt->nodes, node, node_idx_fc);
        int num_empty_leaves = 0;
        int j = 0;
        il_pop_back(&to_process);

        // Loop through the children.
        for (j=0; j < 4; ++j)
        {
            const int child = fc + j;

            // Increment empty leaf count if the child is an empty
            // leaf. Otherwise if the child is a branch, add it to
            // the stack to be processed in the next iteration.
            if (il_get(&qt->nodes, child, node_idx_num) == 0)
                ++num_empty_leaves;
            else if (il_get(&qt->nodes, child, node_idx_num) == -1)
            {
                // Push the child index to the stack.
                il_set(&to_process, il_push_back(&to_process), 0, child);
            }
        }

        // If all the children were empty leaves, remove them and
        // make this node the new empty leaf.
        if (num_empty_leaves == 4)
        {
            // Remove all 4 children in reverse order so that they
            // can be reclaimed on subsequent insertions in proper
            // order.
            il_erase(&qt->nodes, fc + 3);
            il_erase(&qt->nodes, fc + 2);
            il_erase(&qt->nodes, fc + 1);
            il_erase(&qt->nodes, fc + 0);

            // Make this node the new empty leaf.
            il_set(&qt->nodes, node, node_idx_fc, -1);
            il_set(&qt->nodes, node, node_idx_num, 0);
        }
    }
    il_destroy(&to_process);
}

void qt_traverse(Quadtree* qt, void* user_data, QtNodeFunc* branch, QtNodeFunc* leaf)
{
    IntList to_process = {0};
    il_create(&to_process, nd_num);
    push_node(&to_process, 0, 0, qt->root_mx, qt->root_my, qt->root_sx, qt->root_sy);

    while (il_size(&to_process) > 0)
    {
        const int back_idx = il_size(&to_process) - 1;
        const int nd_mx = il_get(&to_process, back_idx, nd_idx_mx);
        const int nd_my = il_get(&to_process, back_idx, nd_idx_my);
        const int nd_sx = il_get(&to_process, back_idx, nd_idx_sx);
        const int nd_sy = il_get(&to_process, back_idx, nd_idx_sy);
        const int nd_index = il_get(&to_process, back_idx, nd_idx_index);
        const int nd_depth = il_get(&to_process, back_idx, nd_idx_depth);
        const int fc = il_get(&qt->nodes, nd_index, node_idx_fc);
        il_pop_back(&to_process);

        if (il_get(&qt->nodes, nd_index, node_idx_num) == -1)
        {
            // Push the children of the branch to the stack.
            const int hx = nd_sx >> 1, hy = nd_sy >> 1;
            const int l = nd_mx-hx, t = nd_my-hx, r = nd_mx+hx, b = nd_my+hy;
            push_node(&to_process, fc+0, nd_depth+1, l,t, hx,hy);
            push_node(&to_process, fc+1, nd_depth+1, r,t, hx,hy);
            push_node(&to_process, fc+2, nd_depth+1, l,b, hx,hy);
            push_node(&to_process, fc+3, nd_depth+1, r,b, hx,hy);
            if (branch)
                branch(qt, user_data, nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy);
        }
        else if (leaf)
            leaf(qt, user_data, nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy);
    }
    il_destroy(&to_process);
}