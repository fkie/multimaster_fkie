# true if massage contains the complete discovered status.
bool full_state

# If full_state is False, the list contains only new or updated participants.
ParticipantEntitiesInfo[] participants

# Removed_participants list is always empty if full_state is True, otherwise the list contains removed participants.
Gid[] removed_participants
