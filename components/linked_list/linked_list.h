/**
* Linked list component. Generic Implementation for all emulators to use as data type for stroing emulation values.
*
* List Contains:
*   - ( void *) to data, to support arbitrary data size (must be allocated on function-independent memory e.g. heap).
*   - ( time_t ) timestamp that dictates activation time of corresponding data.
*   - ( node * ) This pointer points to the next node in the linked list or NULL, if the current node is the last node.
*
* Example: A node could store a void * to a double value that denotes a temperature value in oC (i.e. 25.0 oC).
*          Along with that data, the node stores a timestamp for 12:01:00, while the system time on the ESP32 is 12:00:59.
*          By checking the timestamp of this list element, an emulated temperature sensor knows it has to update its output
*          temperature value in one second to the corresponding value of 25.0 oC.
*/

#ifndef __LINKED_LIST_H__
#define __LINKED_LIST_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// List node structure with data pointer, timestamp and next pointer.
typedef struct list_node_t{
    void *data;
    time_t timestamp;
    struct list_node_t *next;
} list_node_t;

// Callback function signature, called when node is freed.
typedef void ( *free_node_cb) (list_node_t *);

// List structure.
typedef struct list_t{
    size_t length; // Current list length.
    list_node_t *head; // Pointer to current list head.
    list_node_t *tail; // Pointer to current list tail.
    free_node_cb free_cb; // Free callback function pointer for this list.
} list_t;

/**
 * @brief Creates a new list.
 * @param callback      Pointer to free callback function.
*/
list_t *list_new(free_node_cb callback);

/**
 * @brief Frees a lists memory.
 * @param list      Pointer to list.
*/
void list_free(list_t *list);

/**
 * @brief Returns head element of list.
 * @param list      Pointer to list.
*/
list_node_t *list_begin(const list_t *list);

/**
 * @brief Returns tail element of list.
 * @param list      Pointer to list.
*/
list_node_t *list_end(const list_t *list);

/**
 * @brief Appends a new node at the end of a list.
 * @param list      Pointer to list.
 * @param data      Pointer to data stored in new node.
 * @param timestamp Timestamp stored in new node.
*/
bool list_append(list_t *list, void *data, time_t timestamp);

/**
 * @brief Deletes first list element.
 * @param  list     Pointer to list.
 * 
 * @return 'true' on success, else 'false'.
*/
bool list_pop_front(list_t *list);

/**
 * @brief Deletes all elements in list.
 * @param list      Pointer to list.
*/
void list_clear(list_t *list);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LINKED_LIST_H__ */
