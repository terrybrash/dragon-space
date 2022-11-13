class IntList
{
    private int data[] = new int[128];
    private int num_fields = 0;
    private int num = 0;
    private int cap = 128;
    private int free_element = -1;

    // Creates a new list of elements which each consist of integer fields.
    // 'start_num_fields' specifies the number of integer fields each element has.
    public IntList(int start_num_fields)
    {
        num_fields = start_num_fields;
    }

    // Returns the number of elements in the list.
    int size()
    {
        return num;
    }

    // Returns the value of the specified field for the nth element.
    int get(int n, int field)
    {
        assert n >= 0 && n < num && field >= 0 && field < num_fields;
        return data[n*num_fields + field];
    }

    // Sets the value of the specified field for the nth element.
    void set(int n, int field, int val)
    {
        assert n >= 0 && n < num && field >= 0 && field < num_fields;
        data[n*num_fields + field] = val;
    }

    // Clears the list, making it empty.
    void clear()
    {
        num = 0;
        free_element = -1;
    }

    // Inserts an element to the back of the list and returns an index to it.
    int pushBack()
    {
        final int new_pos = (num+1) * num_fields;
        
        // If the list is full, we need to reallocate the buffer to make room
        // for the new element.
        if (new_pos > cap)
        {
            // Use double the size for the new capacity.
            final int new_cap = new_pos * 2;

            // Allocate new array and copy former contents.
            int new_array[] = new int[new_cap];
            System.arraycopy(data, 0, new_array, 0, cap);
            data = new_array;
    
            // Set the old capacity to the new capacity.
            cap = new_cap;
        }
        return num++;
    }
    
    // Removes the element at the back of the list.
    void popBack()
    {
        // Just decrement the list size.
        assert num > 0;
        --num;
    }

    // Inserts an element to a vacant position in the list and returns an index to it.
    int insert()
    {
        // If there's a free index in the free list, pop that and use it.
        if (free_element != -1)
        {
            final int index = free_element;
            final int pos = index * num_fields;
    
            // Set the free index to the next free index.
            free_element = data[pos];
    
            // Return the free index.
            return index;
        }
        // Otherwise insert to the back of the array.
        return pushBack();
    }

    // Removes the nth element in the list.
    void erase(int n)
    {
        // Push the element to the free list.
        final int pos = n * num_fields;
        data[pos] = free_element;
        free_element = n;
    }
}



interface IQtVisitor
{
    // Called when traversing a branch node.
    // (mx, my) indicate the center of the node's AABB.
    // (sx, sy) indicate the half-size of the node's AABB.
    void branch(Quadtree qt, int node, int depth, int mx, int my, int sx, int sy);

    // Called when traversing a leaf node.
    // (mx, my) indicate the center of the node's AABB.
    // (sx, sy) indicate the half-size of the node's AABB.
    void leaf(Quadtree qt, int node, int depth, int mx, int my, int sx, int sy);
}

class Quadtree
{
    // Creates a quadtree with the requested extents, maximum elements per leaf, and maximum tree depth.
    Quadtree(int width, int height, int start_max_elements, int start_max_depth)
    {
        max_elements = start_max_elements;
        max_depth = start_max_depth;

        // Insert the root node to the qt.
        nodes.insert();
        nodes.set(0, node_idx_fc, -1);
        nodes.set(0, node_idx_num, 0);

        // Set the extents of the root node.
        root_mx = width / 2;
        root_my = height / 2;
        root_sx = root_mx;
        root_sy = root_my;
    }

    // Outputs a list of elements found in the specified rectangle.
    public int insert(int id, float x1, float y1, float x2, float y2)
    {
        // Insert a new element.
        final int new_element = elts.insert();
    
        // Set the fields of the new element.
        elts.set(new_element, elt_idx_lft, floor_int(x1));
        elts.set(new_element, elt_idx_top, floor_int(y1));
        elts.set(new_element, elt_idx_rgt, floor_int(x2));
        elts.set(new_element, elt_idx_btm, floor_int(y2));
        elts.set(new_element, elt_idx_id, id);
    
        // Insert the element to the appropriate leaf node(s).
        node_insert(0, 0, root_mx, root_my, root_sx, root_sy, new_element);
        return new_element;
    }

    // Removes the specified element from the tree.
    public void remove(int element)
    {
        // Find the leaves.
        final int lft = elts.get(element, elt_idx_lft);
        final int top = elts.get(element, elt_idx_top);
        final int rgt = elts.get(element, elt_idx_rgt);
        final int btm = elts.get(element, elt_idx_btm);
        IntList leaves = find_leaves(0, 0, root_mx, root_my, root_sx, root_sy, lft, top, rgt, btm);
    
        // For each leaf node, remove the element node.
        for (int j=0; j < leaves.size(); ++j)
        {
            final int nd_index = leaves.get(j, nd_idx_index);
    
            // Walk the list until we find the element node.
            int node_index = nodes.get(nd_index, node_idx_fc);
            int prev_index = -1;
            while (node_index != -1 && enodes.get(node_index, enode_idx_elt) != element)
            {
                prev_index = node_index;
                node_index = enodes.get(node_index, enode_idx_next);
            }
    
            if (node_index != -1)
            {
                // Remove the element node.
                final int next_index = enodes.get(node_index, enode_idx_next);
                if (prev_index == -1)
                    nodes.set(nd_index, node_idx_fc, next_index);
                else
                    enodes.set(prev_index, enode_idx_next, next_index);
                enodes.erase(node_index);

                // Decrement the leaf element count.
                nodes.set(nd_index, node_idx_num, nodes.get(nd_index, node_idx_num)-1);
            }
        }

        // Remove the element.
        elts.erase(element);
    }

    // Cleans up the tree, removing empty leaves.
    public void cleanup()
    {
        IntList to_process = new IntList(1);

        // Only process the root if it's not a leaf.
        if (nodes.get(0, node_idx_num) == -1)
        {
            // Push the root index to the stack.
            to_process.set(to_process.pushBack(), 0, 0);
        }
    
        while (to_process.size() > 0)
        {
            // Pop a node from the stack.
            final int node = to_process.get(to_process.size()-1, 0);
            final int fc = nodes.get(node, node_idx_fc);
            int num_empty_leaves = 0;
            to_process.popBack();
    
            // Loop through the children.
            for (int j=0; j < 4; ++j)
            {
                final int child = fc + j;
    
                // Increment empty leaf count if the child is an empty 
                // leaf. Otherwise if the child is a branch, add it to
                // the stack to be processed in the next iteration.
                if (nodes.get(child, node_idx_num) == 0)
                    ++num_empty_leaves;
                else if (nodes.get(child, node_idx_num) == -1)
                {
                    // Push the child index to the stack.
                    to_process.set(to_process.pushBack(), 0, child);
                }
            }
    
            // If all the children were empty leaves, remove them and 
            // make this node the new empty leaf.
            if (num_empty_leaves == 4)
            {
                // Remove all 4 children in reverse order so that they 
                // can be reclaimed on subsequent insertions in proper
                // order.
                nodes.erase(fc + 3);
                nodes.erase(fc + 2);
                nodes.erase(fc + 1);
                nodes.erase(fc + 0);
    
                // Make this node the new empty leaf.
                nodes.set(node, node_idx_fc, -1);
                nodes.set(node, node_idx_num, 0);
            }
        }
    }

    // Returns a list of elements found in the specified rectangle.
    public IntList query(float x1, float y1, float x2, float y2)
    {
        return query(x1, y1, x2, y2, -1);
    }
    
    // Returns a list of elements found in the specified rectangle excluding the
    // specified element to omit.
    public IntList query(float x1, float y1, float x2, float y2, int omit_element)
    {
        IntList out = new IntList(1);

        // Find the leaves that intersect the specified query rectangle.
        final int qlft = floor_int(x1);
        final int qtop = floor_int(y1);
        final int qrgt = floor_int(x2);
        final int qbtm = floor_int(y2);
        IntList leaves = find_leaves(0, 0, root_mx, root_my, root_sx, root_sy, qlft, qtop, qrgt, qbtm);

        if (temp_size < elts.size())
        {
            temp_size = elts.size();
            temp = new boolean[temp_size];;
        }
    
        // For each leaf node, look for elements that intersect.
        for (int j=0; j < leaves.size(); ++j)
        {
            final int nd_index = leaves.get(j, nd_idx_index);
    
            // Walk the list and add elements that intersect.
            int elt_node_index = nodes.get(nd_index, node_idx_fc);
            while (elt_node_index != -1)
            {
                final int element = enodes.get(elt_node_index, enode_idx_elt);
                final int lft = elts.get(element, elt_idx_lft);
                final int top = elts.get(element, elt_idx_top);
                final int rgt = elts.get(element, elt_idx_rgt);
                final int btm = elts.get(element, elt_idx_btm);
                if (!temp[element] && element != omit_element && intersect(qlft,qtop,qrgt,qbtm, lft,top,rgt,btm))
                {
                    out.set(out.pushBack(), 0, element);
                    temp[element] = true;
                }
                elt_node_index = enodes.get(elt_node_index, enode_idx_next);
            }
        }

        // Unmark the elements that were inserted.
        for (int j=0; j < out.size(); ++j)
            temp[out.get(j, 0)] = false;
        return out;
    }

    // Traverses all the nodes in the tree, calling 'branch' for branch nodes and 'leaf' 
    // for leaf nodes.
    public void traverse(IQtVisitor visitor)
    {
        IntList to_process = new IntList(nd_num);
        pushNode(to_process, 0, 0, root_mx, root_my, root_sx, root_sy);
    
        while (to_process.size() > 0)
        {
            final int back_idx = to_process.size() - 1;
            final int nd_mx = to_process.get(back_idx, nd_idx_mx);
            final int nd_my = to_process.get(back_idx, nd_idx_my);
            final int nd_sx = to_process.get(back_idx, nd_idx_sx);
            final int nd_sy = to_process.get(back_idx, nd_idx_sy);
            final int nd_index = to_process.get(back_idx, nd_idx_index);
            final int nd_depth = to_process.get(back_idx, nd_idx_depth);
            final int fc = nodes.get(nd_index, node_idx_fc);
            to_process.popBack();
    
            if (nodes.get(nd_index, node_idx_num) == -1)
            {
                // Push the children of the branch to the stack.
                final int hx = nd_sx >> 1, hy = nd_sy >> 1;
                final int l = nd_mx-hx, t = nd_my-hx, r = nd_mx+hx, b = nd_my+hy;
                pushNode(to_process, fc+0, nd_depth+1, l,t, hx,hy);
                pushNode(to_process, fc+1, nd_depth+1, r,t, hx,hy);
                pushNode(to_process, fc+2, nd_depth+1, l,b, hx,hy);
                pushNode(to_process, fc+3, nd_depth+1, r,b, hx,hy);
                visitor.branch(this, nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy);
            }
            else
                visitor.leaf(this, nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy);
        }
    }

    private static int floor_int(float val)
    {
        return (int)val;
    }
    
    private static boolean intersect(int l1, int t1, int r1, int b1,
                                     int l2, int t2, int r2, int b2)
    {
        return l2 <= r1 && r2 >= l1 && t2 <= b1 && b2 >= t1;
    }

    private static void pushNode(IntList nodes, int nd_index, int nd_depth, int nd_mx, int nd_my, int nd_sx, int nd_sy)
    {
        final int back_idx = nodes.pushBack();
        nodes.set(back_idx, nd_idx_mx, nd_mx);
        nodes.set(back_idx, nd_idx_my, nd_my);
        nodes.set(back_idx, nd_idx_sx, nd_sx);
        nodes.set(back_idx, nd_idx_sy, nd_sy);
        nodes.set(back_idx, nd_idx_index, nd_index);
        nodes.set(back_idx, nd_idx_depth, nd_depth);
    }

    private IntList find_leaves(int node, int depth, 
                                int mx, int my, int sx, int sy, 
                                int lft, int top, int rgt, int btm)
    {
        IntList leaves = new IntList(nd_num);
        IntList to_process = new IntList(nd_num);
        pushNode(to_process, node, depth, mx, my, sx, sy);
    
        while (to_process.size() > 0)
        {
            final int back_idx = to_process.size() - 1;
            final int nd_mx = to_process.get(back_idx, nd_idx_mx);
            final int nd_my = to_process.get(back_idx, nd_idx_my);
            final int nd_sx = to_process.get(back_idx, nd_idx_sx);
            final int nd_sy = to_process.get(back_idx, nd_idx_sy);
            final int nd_index = to_process.get(back_idx, nd_idx_index);
            final int nd_depth = to_process.get(back_idx, nd_idx_depth);
            to_process.popBack();

            // If this node is a leaf, insert it to the list.
            if (nodes.get(nd_index, node_idx_num) != -1)
                pushNode(leaves, nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy);
            else
            {
                // Otherwise push the children that intersect the rectangle.
                final int fc = nodes.get(nd_index, node_idx_fc);
                final int hx = nd_sx / 2, hy = nd_sy / 2;
                final int l = nd_mx-hx, t = nd_my-hx, r = nd_mx+hx, b = nd_my+hy;
    
                if (top <= nd_my)
                {
                    if (lft <= nd_mx)
                        pushNode(to_process, fc+0, nd_depth+1, l,t,hx,hy);
                    if (rgt > nd_mx)
                        pushNode(to_process, fc+1, nd_depth+1, r,t,hx,hy);
                }
                if (btm > nd_my)
                {
                    if (lft <= nd_mx)
                        pushNode(to_process, fc+2, nd_depth+1, l,b,hx,hy);
                    if (rgt > nd_mx)
                        pushNode(to_process, fc+3, nd_depth+1, r,b,hx,hy);
                }
            }
        }
        return leaves;
    }

    private void node_insert(int index, int depth, int mx, int my, int sx, int sy, int element)
    {
        // Find the leaves and insert the element to all the leaves found.
        final int lft = elts.get(element, elt_idx_lft);
        final int top = elts.get(element, elt_idx_top);
        final int rgt = elts.get(element, elt_idx_rgt);
        final int btm = elts.get(element, elt_idx_btm);
        IntList leaves = find_leaves(index, depth, mx, my, sx, sy, lft, top, rgt, btm);

        for (int j=0; j < leaves.size(); ++j)
        {
            final int nd_mx = leaves.get(j, nd_idx_mx);
            final int nd_my = leaves.get(j, nd_idx_my);
            final int nd_sx = leaves.get(j, nd_idx_sx);
            final int nd_sy = leaves.get(j, nd_idx_sy);
            final int nd_index = leaves.get(j, nd_idx_index);
            final int nd_depth = leaves.get(j, nd_idx_depth);
            leaf_insert(nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy, element);
        }
    }

    private void leaf_insert(int node, int depth, int mx, int my, int sx, int sy, int element)
    {
        // Insert the element node to the leaf.
        final int nd_fc = nodes.get(node, node_idx_fc);
        nodes.set(node, node_idx_fc, enodes.insert());
        enodes.set(nodes.get(node, node_idx_fc), enode_idx_next, nd_fc);
        enodes.set(nodes.get(node, node_idx_fc), enode_idx_elt, element);
    
        // If the leaf is full, split it.
        if (nodes.get(node, node_idx_num) == max_elements && depth < max_depth)
        {
            // Transfer elements from the leaf node to a list of elements.
            IntList elts = new IntList(1);
            while (nodes.get(node, node_idx_fc) != -1)
            {
                final int index = nodes.get(node, node_idx_fc);
                final int next_index = enodes.get(index, enode_idx_next);
                final int elt = enodes.get(index, enode_idx_elt);
    
                // Pop off the element node from the leaf and remove it from the qt.
                nodes.set(node, node_idx_fc, next_index);
                enodes.erase(index);
    
                // Insert element to the list.
                elts.set(elts.pushBack(), 0, elt);
            }
    
            // Start by allocating 4 child nodes.
            final int fc = nodes.insert();
            nodes.insert();
            nodes.insert();
            nodes.insert();
            nodes.set(node, node_idx_fc, fc);
    
            // Initialize the new child nodes.
            for (int j=0; j < 4; ++j)
            {
                nodes.set(fc+j, node_idx_fc, -1);
                nodes.set(fc+j, node_idx_num, 0);
            }
    
            // Transfer the elements in the former leaf node to its new children.
            nodes.set(node, node_idx_num, -1);
            for (int j=0; j < elts.size(); ++j)
                node_insert(node, depth, mx, my, sx, sy, elts.get(j, 0));
        }
        else
        {
            // Increment the leaf element count.
            nodes.set(node, node_idx_num, nodes.get(node, node_idx_num) + 1);
        }
    }


    // ----------------------------------------------------------------------------------------
    // Element node fields:
    // ----------------------------------------------------------------------------------------
    // Points to the next element in the leaf node. A value of -1 
    // indicates the end of the list.
    static final int enode_idx_next = 0;

    // Stores the element index.
    static final int enode_idx_elt = 1;

    // Stores all the element nodes in the quadtree.
    private IntList enodes = new IntList(2);

    // ----------------------------------------------------------------------------------------
    // Element fields:
    // ----------------------------------------------------------------------------------------
    // Stores the rectangle encompassing the element.
    static final int elt_idx_lft = 0, elt_idx_top = 1, elt_idx_rgt = 2, elt_idx_btm = 3;

    // Stores the ID of the element.
    static final int elt_idx_id = 4;

    // Stores all the elements in the quadtree.
    private IntList elts = new IntList(5);

    // ----------------------------------------------------------------------------------------
    // Node fields:
    // ----------------------------------------------------------------------------------------
    // Points to the first child if this node is a branch or the first element
    // if this node is a leaf.
    static final int node_idx_fc = 0;

    // Stores the number of elements in the node or -1 if it is not a leaf.
    static final int node_idx_num = 1;

    // Stores all the nodes in the quadtree. The first node in this
    // sequence is always the root.
    private IntList nodes = new IntList(2);

    // ----------------------------------------------------------------------------------------
    // Node data fields:
    // ----------------------------------------------------------------------------------------
    static final int nd_num = 6;

    // Stores the extents of the node using a centered rectangle and half-size.
    static final int nd_idx_mx = 0, nd_idx_my = 1, nd_idx_sx = 2, nd_idx_sy = 3;

    // Stores the index of the node.
    static final int nd_idx_index = 4;

    // Stores the depth of the node.
    static final int nd_idx_depth = 5;

    // ----------------------------------------------------------------------------------------
    // Data Members
    // ----------------------------------------------------------------------------------------
    // Temporary buffer used for queries.
    private boolean temp[];

    // Stores the size of the temporary buffer.
    private int temp_size = 0;

    // Stores the quadtree extents.
    private int root_mx, root_my, root_sx, root_sy;

    // Maximum allowed elements in a leaf before the leaf is subdivided/split unless
    // the leaf is at the maximum allowed tree depth.
    private int max_elements;

    // Stores the maximum depth allowed for the quadtree.
    private int max_depth;
}
