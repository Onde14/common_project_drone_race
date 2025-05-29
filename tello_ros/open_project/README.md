To spawn N drones add:

`~/CycloneDDS/my-config.xml`:

```
<CycloneDDS>
  <Discovery>
    <ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
  </Discovery>
</CycloneDDS>
```

add `CYCLONEDDS_URI="file://$HOME/CycloneDDS/my-config.xml"` to `~/.bashrc`

Publish the target character to `/char`: `ros2 topic pub /char std_msgs/msg/String "{data: 'X'}"`
