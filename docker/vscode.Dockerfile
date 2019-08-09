FROM nubots:generic

# vscode needs root for some reason
USER root

# Run forever but quit immediatly on a stop command
CMD exec /bin/sh -c "trap : TERM INT; (while true; do sleep 1000; done) & wait"
