/**
 * @file MotionPlanningTree.h
 *
 * @date   Feb 28, 2013
 * @author parallels
 * @brief \todo
 */



#ifndef MOTIONPLANNINGTREE_H_
#define MOTIONPLANNINGTREE_H_

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include <ompl/base/Planner.h>
#include <vector>
//*****************LOCAL DEPENDANCIES**************************//

//**********************NAMESPACES*****************************//

class SimpleMotionPlanningTree
{
public:
	/**
	 * @author Adam Panzica
	 * @brief  Simple single-linked tree node
	 */
	class SimplePathNode
	{
	public:
		/**
		 * Empty constructor
		 */
		SimplePathNode():node_state_(NULL), parent_node_(NULL){};
		/**
		 * @author Adam Panzica
		 * @brief  Constructs a new RTPNode wth a given space information for pre-allocating a state
		 * @param si
		 */
		SimplePathNode(const ompl::base::SpaceInformationPtr& si): node_state_(si->allocState()), parent_node_(NULL){};
		virtual ~SimplePathNode(){};
		ompl::base::State*   node_state_;
		SimplePathNode*      parent_node_;

	};

public:
	SimpleMotionPlanningTree(const ompl::base::SpaceInformationPtr& si);
	virtual ~SimpleMotionPlanningTree();

	typedef std::vector<SimplePathNode*> tree_t;
	typedef tree_t::size_type size_type;

	/**
	 * @author Adam Panzica
	 * @return The size of the tree
	 */
	size_type size() const;

	/**
	 * @author Adam Panzica
	 * @brief  Adds a node to the tree
	 * @param [in] node The node to add to the tree
	 * @param [in] index The index of the added node on the tree
	 */
	void addNode(SimplePathNode* node, size_type& index);


	/**
	 * @author Adam Panzica
	 * @brief  Creates a new node on the tree
	 * @param [out] index The index in the tree of the added node
	 * @return A pointer to the newly created node
	 */
	SimplePathNode* addNode(size_type& index);

	/**
	 * @author Adam Panzica
	 * @brief  Gets a node in the tree by index
	 * @param index The index of the node to access
	 * @return Pointer to the node at the given index, or NULL if the index was invalid
	 */
	SimplePathNode* getNode(size_type index) const;

	/**
	 * @author Adam Panzica
	 * @brief  Frees the memory for all nodes contained in the tree and clears the tree
	 *
	 * Note that this will invalidate any pointers that have been returned by the addNode and getNode methods
	 */
	void flushTree();
private:
	tree_t nodes_;
	ompl::base::SpaceInformationPtr si_;
};


#endif /* MOTIONPLANNINGTREE_H_ */
