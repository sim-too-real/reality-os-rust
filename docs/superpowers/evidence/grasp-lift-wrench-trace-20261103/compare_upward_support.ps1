$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..')).Path
$freezePath = Join-Path $PSScriptRoot 'FREEZE.json'
$resultPath = Join-Path $PSScriptRoot 'RESULT.json'
$modelPath = Join-Path $repoRoot 'robots/bundles/arm_gripper_lift/model.xml'
$reportPath = Join-Path $PSScriptRoot 'UPWARD_SUPPORT_COMPARISON.json'

function Assert-Condition([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw $Message }
}

function Get-Sha256([string]$Path) {
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToUpperInvariant()
}

function Get-FiniteNumber($Value, [string]$Name) {
    Assert-Condition ($null -ne $Value) "$Name is missing"
    $number = [double]$Value
    Assert-Condition (-not [double]::IsNaN($number) -and -not [double]::IsInfinity($number)) "$Name is non-finite"
    return $number
}

$freeze = Get-Content -LiteralPath $freezePath -Raw | ConvertFrom-Json
$result = Get-Content -LiteralPath $resultPath -Raw | ConvertFrom-Json
$freezeHash = Get-Sha256 $freezePath
$resultHash = Get-Sha256 $resultPath
Assert-Condition ($result.freeze_sha256 -eq $freezeHash) 'result is not bound to the committed freeze'

[xml]$model = Get-Content -LiteralPath $modelPath -Raw
$gravity = @($model.mujoco.option.gravity.Split(' ') | ForEach-Object { [double]::Parse($_, [Globalization.CultureInfo]::InvariantCulture) })
Assert-Condition ($gravity.Count -eq 3 -and $gravity[2] -lt 0.0) 'expected a downward world-z gravity vector'
$actuatorNames = @($model.mujoco.actuator.ChildNodes | ForEach-Object { $_.GetAttribute('name') })
Assert-Condition ($actuatorNames.Count -ge 6 -and $actuatorNames[4] -eq 'grip_l' -and $actuatorNames[5] -eq 'grip_r') 'frozen model actuator ordering does not match the two finger channels'

$trialReports = @()
foreach ($binding in $freeze.primary_control_inputs) {
    $inputPath = Join-Path $repoRoot $binding.path
    Assert-Condition ((Get-Sha256 $inputPath) -eq $binding.sha256) "trial input hash mismatch: $($binding.trial_id)"
    $trialBundle = Get-Content -LiteralPath $inputPath -Raw | ConvertFrom-Json
    Assert-Condition ($trialBundle.trial.trial_id -eq $binding.trial_id) 'input trial id mismatch'
    $episodeMatches = @($result.records | Where-Object { $_.episode.trial_id -eq $binding.trial_id })
    Assert-Condition ($episodeMatches.Count -eq 1) "missing/duplicate replay record: $($binding.trial_id)"
    $episode = $episodeMatches[0].episode
    $object = @($trialBundle.trial.scenario.objects | Where-Object { $_.name -eq $trialBundle.trial.scenario.object_id })
    Assert-Condition ($object.Count -eq 1) 'frozen object mass is missing or ambiguous'
    $mass = Get-FiniteNumber $object[0].mass 'object mass'
    $weight = $mass * [Math]::Abs($gravity[2])
    $muLower = Get-FiniteNumber $episode.grasp_simulated_friction_oracle_interval[0] 'simulated friction lower bound'

    $postCloseRows = @($episode.grasp_contact_audits | Where-Object { $_.phase -eq 'post_close' })
    $firstPostClosePair = @($postCloseRows | Select-Object -First 2)
    $firstLiftRows = @($episode.grasp_contact_audits | Where-Object { $_.phase -eq 'lift_waypoint_1' })
    Assert-Condition ($firstPostClosePair.Count -eq 2 -and $firstLiftRows.Count -eq 2) "required post-close/first-lift samples missing: $($binding.trial_id)"
    Assert-Condition ($firstPostClosePair[0].finger_body -ne $firstPostClosePair[1].finger_body) 'post-close finger rows are not bilateral'
    Assert-Condition ($firstLiftRows[0].finger_body -ne $firstLiftRows[1].finger_body) 'first-lift finger rows are not bilateral'

    $normalSum = 0.0
    foreach ($row in $firstPostClosePair) { $normalSum += Get-FiniteNumber $row.normal_force_n 'post-close summed contact normal' }
    $scalarCapacityProxy = $muLower * $normalSum
    $postCloseVertical = 0.0
    foreach ($row in $firstPostClosePair) {
        foreach ($contact in $row.contact_wrenches) {
            $postCloseVertical += Get-FiniteNumber $contact.force_world_on_object_n[2] 'post-close world-z contact force'
        }
    }

    $firstLiftVertical = 0.0
    $netMoment = @(0.0, 0.0, 0.0)
    $firstLiftContactCount = 0
    foreach ($row in $firstLiftRows) {
        foreach ($contact in $row.contact_wrenches) {
            $force = $contact.force_world_on_object_n
            $moment = $contact.moment_world_about_object_com_nm
            Assert-Condition ($null -ne $force -and $force.Count -eq 3) 'first-lift derived world force missing'
            Assert-Condition ($null -ne $moment -and $moment.Count -eq 3) 'first-lift moment about COM missing'
            $firstLiftVertical += Get-FiniteNumber $force[2] 'first-lift world-z contact force'
            for ($axis = 0; $axis -lt 3; $axis++) { $netMoment[$axis] += Get-FiniteNumber $moment[$axis] 'first-lift moment component' }
            $firstLiftContactCount++
        }
    }
    Assert-Condition ($firstLiftContactCount -gt 0) 'first lift has no object-finger contacts'

    $actuatorForce = $firstLiftRows[0].actuator_force
    Assert-Condition ($actuatorForce.Count -eq $actuatorNames.Count) 'actuator force vector does not match frozen model ordering'
    $fingerActuatorForces = @(
        (Get-FiniteNumber $actuatorForce[4] 'grip_l actuator force'),
        (Get-FiniteNumber $actuatorForce[5] 'grip_r actuator force')
    )

    $postCloseDuplicatePair = $null
    if ($postCloseRows.Count -eq 4) {
        $firstPairSignature = @($postCloseRows[0], $postCloseRows[1] | ForEach-Object {
            [ordered]@{ finger_body = $_.finger_body; normal_force_n = $_.normal_force_n; ctrl = $_.ctrl; actuator_force = $_.actuator_force; contact_wrenches = $_.contact_wrenches }
        }) | ConvertTo-Json -Depth 12 -Compress
        $secondPairSignature = @($postCloseRows[2], $postCloseRows[3] | ForEach-Object {
            [ordered]@{ finger_body = $_.finger_body; normal_force_n = $_.normal_force_n; ctrl = $_.ctrl; actuator_force = $_.actuator_force; contact_wrenches = $_.contact_wrenches }
        }) | ConvertTo-Json -Depth 12 -Compress
        $postCloseDuplicatePair = ($firstPairSignature -eq $secondPairSignature)
    }

    $trialReports += [ordered]@{
        trial_id = $episode.trial_id
        outcome = $episode.outcome
        failure_cause = $episode.failure_cause
        input_sha256 = $binding.sha256
        model_hash = $trialBundle.trial.model_hash
        mass_kg = $mass
        gravity_world_m_s2 = $gravity
        object_weight_n = $weight
        simulated_friction_interval = $episode.grasp_simulated_friction_oracle_interval
        post_close_normal_sum_n = $normalSum
        scalar_capacity_proxy_n = $scalarCapacityProxy
        scalar_proxy_to_weight_ratio = $scalarCapacityProxy / $weight
        post_close_realized_vertical_contact_force_n = $postCloseVertical
        first_lift_phase = 'lift_waypoint_1'
        first_lift_object_finger_contacts = $firstLiftContactCount
        first_lift_realized_vertical_contact_force_n = $firstLiftVertical
        first_lift_weight_deficit_n = $weight - $firstLiftVertical
        first_lift_realized_force_to_weight_ratio = $firstLiftVertical / $weight
        first_lift_net_moment_about_object_com_nm = $netMoment
        first_lift_grip_l_actuator_force_n = $fingerActuatorForces[0]
        first_lift_grip_r_actuator_force_n = $fingerActuatorForces[1]
        repeated_post_close_pair_matches = $postCloseDuplicatePair
    }
}

$report = [ordered]@{
    schema = 'realityos.grasp_lift_upward_support_comparison/v1'
    generated_at_utc = [DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ')
    classification = 'OFFLINE_SIMULATION_TRACE_ANALYSIS; TWO_FROZEN_DEVELOPMENT_CONTROLS; NOT_ADMITTED'
    freeze_sha256 = $freezeHash
    result_sha256 = $resultHash
    analysis_script_sha256 = Get-Sha256 $PSCommandPath
    model_xml_sha256 = Get-Sha256 $modelPath
    model_xml_path = 'robots/bundles/arm_gripper_lift/model.xml'
    method = 'Compare the prior scalar lower-friction-times-post-close-normal proxy with the signed world-z resultant of object-finger contact forces at the first lift waypoint; compute weight from frozen object mass and the checked-in MuJoCo gravity setting. Sum per-contact moments about object COM at that waypoint.'
    decision = 'Keep the 1.95 N finger-force cap and all existing hold/refusal gates unchanged. Do not treat the scalar proxy as sufficient for lift acceptance; next evaluate a direction- and moment-aware offline predictor before any additional full-lift replay.'
    limitations = @(
        'Only two development controls were analyzed; this does not validate predictive performance or generalize to held-out cases.',
        'The first-lift contact resultant is simulated contact evidence, not physical force measurement or hardware capacity.',
        'The post-close phase has no sample timestamp; where duplicate rows exist, their sequence is retained and equality is reported.',
        'The world-z finger contact resultant excludes the table reaction and should not be interpreted as total object force balance.'
    )
    trials = $trialReports
}
$json = $report | ConvertTo-Json -Depth 16
[System.IO.File]::WriteAllText($reportPath, $json, [System.Text.UTF8Encoding]::new($false))
Write-Output "SUPPORT_COMPARISON_OK report=$reportPath result=$resultHash"
Write-Output ($trialReports | ConvertTo-Json -Depth 8)
