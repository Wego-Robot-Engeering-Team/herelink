# Rover Metadata Notes

PDF `22.Metadata - Rover 명세.pdf` 기준으로 정리했습니다.

현재 파일:

- `rover_metadata_example.json`: PDF에 나온 필드를 모두 넣은 샘플
- `validate_rover_metadata.py`: 필수 필드 검사 + 선택 필드가 들어온 경우 타입 검사

필드 정리:

## ius Object

| key | required |
| --- | --- |
| metaVersion | Y |
| appVersion | Y |
| draftInspectionUnitId | Y |
| drone | Y |
| startTime | Y |
| completeTime | Y |
| missionUuid | Y |
| bladePosMap | Y |
| chamberPosMap | Y |
| remark | Y |
| photos | Y |
| videos | Y |
| lidarData | Y |

## bladePosMap Object

| key | required | value |
| --- | --- | --- |
| 0 | Y | A/B/C/1/2/3 |
| 1 | Y | A/B/C/1/2/3 |
| 2 | Y | A/B/C/1/2/3 |

## chamberPosMap Object

| key | required | value |
| --- | --- | --- |
| 0 | Y | ACC/LE/CC/TE/TE2 |
| 1 | Y | ACC/LE/CC/TE/TE2 |
| 2 | Y | ACC/LE/CC/TE/TE2 |
| 3 | Y | ACC/LE/CC/TE/TE2 |
| 4 | Y | ACC/LE/CC/TE/TE2 |

## photos Object

| key | required |
| --- | --- |
| fileDir | Y |
| filename | Y |
| photoTakenAt | Y |
| uniqueKey | Y |
| chamberPosition | Y |
| bladePosition | Y |
| timeZone | Y |
| sequenceId | Y |
| n | N |
| e | N |
| alt | N |
| r | Y |
| bodyRoll | N |
| bodyPitch | N |
| bodyYaw | N |
| gimbalRoll | Y |
| gimbalPitch | Y |
| gimbalYaw | Y |
| gpsAltitude | Y |
| gpsLatitude | Y |
| gpsLongitude | Y |
| focalLength | Y |
| fNumber | Y |
| measuredDistanceToSurface | Y |
| isManualShoot | Y |
| ledPower | Y |

## videos Object

| key | required |
| --- | --- |
| fileDir | Y |
| filename1 | Y |
| filename2 | N |
| videoTakenAt | Y |
| videoLength | Y |
| uniqueKey | Y |
| chamberPosition | Y |
| bladePosition | Y |
| timeZone | Y |
| sequenceId | Y |
| n | N |
| e | N |
| alt | N |
| r | Y |
| bodyRoll | N |
| bodyPitch | N |
| bodyYaw | N |
| gpsAltitude | Y |
| gpsLatitude | Y |
| gpsLongitude | Y |
| ledPower | Y |

## lidarData Object

| key | required |
| --- | --- |
| lidarTakenAt | Y |
| lidarLength | Y |
| uniqueKey | Y |
| chamberPosition | Y |
| bladePosition | Y |
| sequenceId | Y |
| r | Y |
| resolutionVertical | Y |
| resolutionHorizontal | Y |
| timeStep | Y |
| distanceToSurfaces | N |

메모:

- `draftInspectionUnitId`, `measuredDistanceToSurface`, `resolutionVertical`, `resolutionHorizontal`, `distanceToSurfaces` 는 PDF 줄바꿈을 자연스럽게 이어 붙여 적었습니다.
- 샘플 JSON에는 선택 항목도 같이 넣었습니다.
