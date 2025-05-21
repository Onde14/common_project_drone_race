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