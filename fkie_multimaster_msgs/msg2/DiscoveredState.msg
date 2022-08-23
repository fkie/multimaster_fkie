# the timestamp of this state
uint64 stamp
ParticipantEntitiesInfo[] participants

# Removed entities since last report
# since the status with the timestamp
uint64 since
# removed nodes since last_message. Remove all nodes of the participant if node_entities is empty.
# enclave, unicast_locators, topic_entities and publisher, subscriber of the underlined messages
# are not valid!
ParticipantEntitiesInfo[] removed_nodes
# removed topic IDs
Guid[] removed_topics