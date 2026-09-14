# Security Notes

Before publishing or sharing a fork:

- Remove real Wi-Fi credentials.
- Remove real TURN usernames, passwords, and domains.
- Remove private DNS names, personal hostnames, and public IPs.
- Do not commit `sdkconfig` if it contains private deployment values.
- Do not commit build outputs.
- Review logs before sharing; they may contain remote IP addresses.

This public bundle already replaces the known TURN values with placeholders.
