#include <stdio.h>
#include "linked_list.h"

list_t *list_new(free_node_cb callback) {
    // Allocate memory for list struct.
    list_t *list = (list_t *) calloc(1, sizeof(list_t));
    if(!list) return NULL;
    list->head = list->tail = NULL;
    list->length = 0;
    list->free_cb = callback;
    return list;
}

void list_free(list_t *list) {
    if(!list) return;
    // Clear list before free, if list is not clear already.
    list_clear(list);
    free(list);
}

list_node_t *list_begin(const list_t *list) {
    if(!list) NULL;
    return list->head;
}

list_node_t *list_end(const list_t *list) {
    if(!list) NULL;
    return list->tail;
}

bool list_append(list_t *list, void *data, time_t timestamp) {
    if(!list) return false;
    if(!data) return false;

    list_node_t *node = (list_node_t *) calloc(1, sizeof(list_node_t));
    if(!node) return false;
    node->next = NULL;
    node->data = data;
    node->timestamp = timestamp;

    if(!list->tail) {
        list->head = node;
        list->tail = node;
    } else {
        list->tail->next = node;
        list->tail = node;
    }
    ++list->length;
    return true;
}

bool list_pop_front(list_t *list) {
    if(!list) return false;

    if(list->head) {
        list_node_t *node = list->head;
        list->head = list->head->next;
        // Since pop_front is only way to remove list nodes, free callback of node is only needed here.
        list->free_cb(node);
        free(node);
        --list->length;
        return true;
    } else {
        return false;
    }
}

void list_clear(list_t *list) {
    if(!list) return;
    // Pop front on all list nodes until list is empty.
    while(list->head) list_pop_front(list);
    list->head = NULL;
    list->tail = NULL;
    list->length = 0;
}